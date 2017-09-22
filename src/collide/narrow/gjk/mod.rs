pub use self::epa::{EPA2, EPA3};
pub use self::simplex::{SimplexProcessor2, SimplexProcessor3};

use std;
use std::fmt::Debug;
use std::ops::Neg;

use cgmath::prelude::*;
use collision::Discrete;

use self::epa::EPA;
use self::simplex::SimplexProcessor;
use super::NarrowPhase;
use {Pose, Real};
use collide::{CollisionShape, CollisionStrategy, Contact, ContactSet, Primitive};
use collide::primitives::SupportFunction;

mod simplex;
mod epa;

const MAX_ITERATIONS: u32 = 100;

#[derive(Debug)]
struct RunningAverage {
    pub average: f64,
    n: u32,
}

impl RunningAverage {
    pub fn new() -> Self {
        Self { average: 0., n: 0 }
    }

    pub fn add(&mut self, value: u32) {
        let n0 = self.n as f64;
        self.n += 1;
        let n = self.n as f64;
        self.average *= n0 / n;
        self.average += (value as f64) / n;
    }
}

/// Gilbert-Johnson-Keerthi narrow phase collision detection algorithm.
///
/// # Type parameters:
///
/// - `P`: collision primitive type.
/// - `T`: transform type
/// - `S`: simplex processor type. Should be either
///        [`SimplexProcessor2`](struct.SimplexProcessor2.html) or
///        [`SimplexProcessor3`](struct.SimplexProcessor3.html)
/// - `E`: EPA algorithm implementation type. Should be either
///        [`EPA2`](struct.EPA2.html) or
///        [`EPA3`](struct.EPA3.html)
///
#[derive(Debug)]
pub struct GJK<P, T, S, E> {
    simplex_processor: S,
    epa: E,
    average: RunningAverage,
    marker: std::marker::PhantomData<(P, T)>,
}

impl<P, T, S, E> GJK<P, T, S, E>
where
    P: Primitive,
    S: SimplexProcessor<Point = P::Point>,
    T: Pose<P::Point>,
    E: EPA<P, T>,
{
    /// Create a new GJK algorithm implementation
    pub fn new() -> Self {
        Self {
            simplex_processor: S::new(),
            average: RunningAverage::new(),
            epa: E::new(),
            marker: std::marker::PhantomData,
        }
    }

    /// Return the average amount of iterations run by the algorithm for each collision pair.
    pub fn get_average(&self) -> f64 {
        self.average.average
    }
}

impl<ID, P, T, S, E> NarrowPhase<ID, P, T> for GJK<P, T, S, E>
where
    ID: Debug + Clone,
    P: Primitive,
    P::Aabb: Discrete<P::Aabb>,
    P::Point: Debug,
    <P::Point as EuclideanSpace>::Diff: InnerSpace
        + Neg<Output = <P::Point as EuclideanSpace>::Diff> + Debug,
    S: SimplexProcessor<Point = P::Point> + Debug,
    T: Pose<P::Point> + Debug,
    E: EPA<P, T> + Debug,
{
    fn collide(
        &mut self,
        (ref left_id, ref left, ref left_transform): (ID, &CollisionShape<P, T>, &T),
        (ref right_id, ref right, ref right_transform): (ID, &CollisionShape<P, T>, &T),
    ) -> Option<ContactSet<ID, P::Point>> {
        if !left.enabled || !right.enabled || left.primitives.is_empty() ||
            right.primitives.is_empty()
        {
            return None;
        }

        let mut contacts = Vec::default();
        for &(ref left_primitive, ref left_local_transform) in &left.primitives {
            let left_transform = left_transform.concat(left_local_transform);
            for &(ref right_primitive, ref right_local_transform) in &right.primitives {
                let right_transform = right_transform.concat(right_local_transform);
                match gjk(
                    left_primitive,
                    &left_transform,
                    right_primitive,
                    &right_transform,
                    &self.simplex_processor,
                    &mut self.average,
                ) {
                    Some(mut simplex) => {
                        if left.strategy == CollisionStrategy::CollisionOnly ||
                            right.strategy == CollisionStrategy::CollisionOnly
                        {
                            contacts.push((Contact::new(CollisionStrategy::CollisionOnly)));
                        } else {
                            contacts.append(&mut self.epa.process(
                                &mut simplex,
                                left_primitive,
                                &left_transform,
                                right_primitive,
                                &right_transform,
                            ));
                        }
                    }
                    None => (),
                };
            }
        }

        if contacts.len() > 0 {
            Some(ContactSet::new(
                (left_id.clone(), right_id.clone()),
                contacts,
            ))
        } else {
            None
        }
    }
}

fn gjk<P, T, S>(
    left: &P,
    left_transform: &T,
    right: &P,
    right_transform: &T,
    simplex_processor: &S,
    average: &mut RunningAverage,
) -> Option<Vec<SupportPoint<P::Point>>>
where
    P: SupportFunction,
    P::Point: EuclideanSpace<Scalar = Real>,
    <P::Point as EuclideanSpace>::Diff: Neg<Output = <P::Point as EuclideanSpace>::Diff> + InnerSpace,
    T: Pose<P::Point>,
    S: SimplexProcessor<Point = P::Point>,
{
    let mut d = *right_transform.position() - *left_transform.position();
    let a = support(left, left_transform, right, right_transform, &d);
    if a.v.dot(d) <= 0. {
        average.add(0);
        return None;
    }
    let mut simplex: Vec<SupportPoint<P::Point>> = Vec::default();
    simplex.push(a);
    d = d.neg();
    let mut i = 0;
    loop {
        let a = support(left, left_transform, right, right_transform, &d);
        if a.v.dot(d) <= 0. {
            average.add(i + 1);
            return None;
        } else {
            simplex.push(a);
            if simplex_processor.check_origin(&mut simplex, &mut d) {
                average.add(i + 1);
                return Some(simplex);
            }
        }
        i += 1;
        if i >= MAX_ITERATIONS {
            average.add(i);
            return None;
        }
    }
}

#[derive(Clone, Debug)]
pub struct SupportPoint<P>
where
    P: EuclideanSpace,
{
    v: P::Diff,
    sup_a: P,
    sup_b: P,
}

impl<P> SupportPoint<P>
where
    P: EuclideanSpace<Scalar = Real>,
{
    pub fn new() -> Self {
        Self {
            v: P::Diff::zero(),
            sup_a: P::from_value(0.),
            sup_b: P::from_value(0.),
        }
    }
}

pub(crate) fn support<S, T>(
    left: &S,
    left_transform: &T,
    right: &S,
    right_transform: &T,
    direction: &<S::Point as EuclideanSpace>::Diff,
) -> SupportPoint<S::Point>
where
    S: SupportFunction,
    S::Point: EuclideanSpace<Scalar = Real>,
    <S::Point as EuclideanSpace>::Diff: Neg<Output = <S::Point as EuclideanSpace>::Diff>,
    T: Pose<S::Point>,
{
    let l = left.support_point(direction, left_transform);
    let r = right.support_point(&direction.neg(), right_transform);
    SupportPoint {
        v: l - r,
        sup_a: l,
        sup_b: r,
    }
}

#[cfg(test)]
mod tests {
    use cgmath::{Point2, Point3, Quaternion, Rad, Rotation2, Rotation3, Vector2};

    use super::{gjk, support, RunningAverage};
    use super::simplex::{SimplexProcessor, SimplexProcessor2, SimplexProcessor3};
    use Real;
    use collide::narrow::NarrowPhase;
    use collide2d::*;
    use collide3d::*;

    fn transform(x: Real, y: Real, angle: Real) -> BodyPose2 {
        BodyPose2::new(Point2::new(x, y), Rotation2::from_angle(Rad(angle)))
    }

    fn transform_3d(x: Real, y: Real, z: Real, angle_z: Real) -> BodyPose3 {
        BodyPose3::new(Point3::new(x, y, z), Quaternion::from_angle_z(Rad(angle_z)))
    }

    #[test]
    fn test_support() {
        let left = Rectangle::new(10., 10.);
        let left_transform = transform(15., 0., 0.);
        let right = Rectangle::new(10., 10.);
        let right_transform = transform(-15., 0., 0.);
        let direction = Vector2::new(1., 0.);
        assert_eq!(
            Vector2::new(40., 0.),
            support(&left, &left_transform, &right, &right_transform, &direction).v
        );
    }

    #[test]
    fn test_gjk_miss() {
        let left = Rectangle::new(10., 10.);
        let left_transform = transform(15., 0., 0.);
        let right = Rectangle::new(10., 10.);
        let right_transform = transform(-15., 0., 0.);
        let processor = SimplexProcessor2::new();
        let mut average = RunningAverage::new();
        assert!(
            gjk(
                &left,
                &left_transform,
                &right,
                &right_transform,
                &processor,
                &mut average,
            ).is_none()
        );
    }

    #[test]
    fn test_gjk_hit() {
        let left = Rectangle::new(10., 10.);
        let left_transform = transform(15., 0., 0.);
        let right = Rectangle::new(10., 10.);
        let right_transform = transform(7., 2., 0.);
        let processor = SimplexProcessor2::new();
        let mut average = RunningAverage::new();
        let simplex = gjk(
            &left,
            &left_transform,
            &right,
            &right_transform,
            &processor,
            &mut average,
        );
        assert!(simplex.is_some());
    }

    #[test]
    fn test_gjk_3d_hit() {
        let left = Cuboid::new(10., 10., 10.);
        let left_transform = transform_3d(15., 0., 0., 0.);
        let right = Cuboid::new(10., 10., 10.);
        let right_transform = transform_3d(7., 2., 0., 0.);
        let processor = SimplexProcessor3::new();
        let mut average = RunningAverage::new();
        let simplex = gjk(
            &left,
            &left_transform,
            &right,
            &right_transform,
            &processor,
            &mut average,
        );
        assert!(simplex.is_some());
    }

    #[test]
    fn test_gjk_shape_simple_miss() {
        let left = CollisionShape2::new_simple(
            CollisionStrategy::CollisionOnly,
            Rectangle::new(10., 10.).into(),
        );
        let left_transform = transform(15., 0., 0.);
        let right = CollisionShape2::new_simple(
            CollisionStrategy::CollisionOnly,
            Rectangle::new(10., 10.).into(),
        );
        let right_transform = transform(-15., 0., 0.);
        let mut gjk = GJK2::new();
        assert!(
            gjk.collide((1, &left, &left_transform), (2, &right, &right_transform))
                .is_none()
        );
    }

    #[test]
    fn test_gjk_shape_simple_hit() {
        let left = CollisionShape2::new_simple(
            CollisionStrategy::CollisionOnly,
            Rectangle::new(10., 10.).into(),
        );
        let left_transform = transform(15., 0., 0.);
        let right = CollisionShape2::new_simple(
            CollisionStrategy::CollisionOnly,
            Rectangle::new(10., 10.).into(),
        );
        let right_transform = transform(7., 2., 0.);
        let mut gjk = GJK2::new();
        let set = gjk.collide((1, &left, &left_transform), (2, &right, &right_transform));
        assert!(set.is_some());
        let contact_set = set.unwrap();
        assert_eq!((1, 2), contact_set.bodies);
        assert_eq!(1, contact_set.contacts.len());
    }

    #[test]
    fn test_gjk_3d_shape_hit() {
        let left = CollisionShape3::new_simple(
            CollisionStrategy::CollisionOnly,
            Cuboid::new(10., 10., 10.).into(),
        );
        let left_transform = transform_3d(15., 0., 0., 0.);
        let right = CollisionShape3::new_simple(
            CollisionStrategy::CollisionOnly,
            Cuboid::new(10., 10., 10.).into(),
        );
        let right_transform = transform_3d(7., 2., 0., 0.);
        let mut gjk = GJK3::new();
        let set = gjk.collide((1, &left, &left_transform), (2, &right, &right_transform));
        assert!(set.is_some());
        let contact_set = set.unwrap();
        assert_eq!((1, 2), contact_set.bodies);
        assert_eq!(1, contact_set.contacts.len());
    }

    #[test]
    fn test_gjk_shape_complex_miss() {
        let left = CollisionShape2::new_complex(
            CollisionStrategy::CollisionOnly,
            vec![
                (
                    Rectangle::new(10., 10.).into(),
                    transform(0., 5., 0.)
                ),
                (
                    Rectangle::new(10., 10.).into(),
                    transform(0., -5., 0.)
                ),
            ],
        );
        let left_transform = transform(15., 0., 0.);
        let right = CollisionShape2::new_simple(
            CollisionStrategy::CollisionOnly,
            Rectangle::new(10., 10.).into(),
        );
        let right_transform = transform(-15., 0., 0.);
        let mut gjk = GJK2::new();
        assert!(
            gjk.collide((1, &left, &left_transform), (2, &right, &right_transform))
                .is_none()
        );
    }

    #[test]
    fn test_gjk_shape_complex_hit_single() {
        let left = CollisionShape2::new_complex(
            CollisionStrategy::CollisionOnly,
            vec![
                (
                    Rectangle::new(10., 10.).into(),
                    transform(0., 5., 0.)
                ),
                (
                    Rectangle::new(10., 10.).into(),
                    transform(0., -5., 0.)
                ),
            ],
        );
        let left_transform = transform(15., 0., 0.);
        let right = CollisionShape2::new_simple(
            CollisionStrategy::CollisionOnly,
            Rectangle::new(10., 10.).into(),
        );
        let right_transform = transform(7., 6., 0.);
        let mut gjk = GJK2::new();
        let set = gjk.collide((1, &left, &left_transform), (2, &right, &right_transform));
        assert!(set.is_some());
        let contact_set = set.unwrap();
        assert_eq!((1, 2), contact_set.bodies);
        assert_eq!(1, contact_set.contacts.len());
    }

    #[test]
    fn test_gjk_shape_complex_hit_both() {
        let left = CollisionShape2::new_complex(
            CollisionStrategy::CollisionOnly,
            vec![
                (
                    Rectangle::new(10., 10.).into(),
                    transform(0., 5., 0.)
                ),
                (
                    Rectangle::new(10., 10.).into(),
                    transform(0., -5., 0.)
                ),
            ],
        );
        let left_transform = transform(15., 0., 0.);
        let right = CollisionShape2::new_simple(
            CollisionStrategy::CollisionOnly,
            Rectangle::new(10., 10.).into(),
        );
        let right_transform = transform(7., 4., 0.);
        let mut gjk = GJK2::new();
        let set = gjk.collide((1, &left, &left_transform), (2, &right, &right_transform));
        assert!(set.is_some());
        let contact_set = set.unwrap();
        assert_eq!((1, 2), contact_set.bodies);
        assert_eq!(2, contact_set.contacts.len());
    }
}
