pub use self::epa::{EPA2D, EPA3D};
pub use self::simplex::{SimplexProcessor2D, SimplexProcessor3D};

use std;
use std::fmt::Debug;
use std::ops::Neg;

use cgmath::prelude::*;
use collision::Discrete;

use self::epa::EPA;
use self::simplex::SimplexProcessor;
use super::NarrowPhase;
use {Pose, Real};
use collide::{Contact, ContactSet, CollisionShape, CollisionStrategy, CollisionPrimitive,
              Primitive};

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
///        [`SimplexProcessor2D`](struct.SimplexProcessor2D.html) or
///        [`SimplexProcessor3D`](struct.SimplexProcessor3D.html)
/// - `E`: EPA algorithm implementation type. Should be either
///        [`EPA2D`](struct.EPA2D.html) or
///        [`EPA3D`](struct.EPA3D.html)
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
    P: EuclideanSpace<Scalar = Real>,
    S: SimplexProcessor<Vector = P::Diff>,
    T: Pose<P>,
    E: EPA<T>,
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

impl<ID, P, T, S, E> NarrowPhase<ID, P, T> for GJK<P::Point, T, S, E>
    where
        ID: Debug + Clone,
        P: Primitive,
        P::Aabb: Discrete<P::Aabb>,
        P::Point: Debug,
        P::Vector: InnerSpace + Neg<Output=P::Vector>,
        S: SimplexProcessor<Vector=P::Vector, Point=P::Point> + Debug,
        T: Pose<P::Point> + Debug,
        E: EPA<T, Vector=P::Vector, Primitive=P, Point=P::Point> + Debug,
{
    fn collide(
        &mut self,
        (ref left_id, ref left, ref left_transform): (ID, &CollisionShape<P, T>, &T),
        (ref right_id, ref right, ref right_transform): (ID, &CollisionShape<P, T>, &T),
    ) -> Option<ContactSet<ID, P::Vector>> {
        if !left.enabled || !right.enabled || left.primitives.is_empty() ||
            right.primitives.is_empty()
            {
                return None;
            }

        let mut contacts = Vec::default();
        for left_primitive in &left.primitives {
            for right_primitive in &right.primitives {
                if left.transformed_bound.intersects(&right.transformed_bound) {
                    match gjk(
                        &left_primitive,
                        left_transform,
                        &right_primitive,
                        right_transform,
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
                                    left_transform,
                                    right_primitive,
                                    right_transform,
                                ));
                            }
                        }
                        None => (),
                    };
                }
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
    left: &CollisionPrimitive<P, T>,
    left_transform: &T,
    right: &CollisionPrimitive<P, T>,
    right_transform: &T,
    simplex_processor: &S,
    average: &mut RunningAverage,
) -> Option<Vec<SupportPoint<P::Point>>>
where
    P::Vector: Neg<Output = P::Vector> + InnerSpace,
    P: Primitive,
    T: Pose<P::Point>,
    S: SimplexProcessor<Vector = P::Vector, Point = P::Point>,
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

pub(crate) fn support<P, T>(
    left: &CollisionPrimitive<P, T>,
    left_transform: &T,
    right: &CollisionPrimitive<P, T>,
    right_transform: &T,
    direction: &P::Vector,
) -> SupportPoint<P::Point>
where
    P: Primitive,
    T: Pose<P::Point>,
    P::Vector: Neg<Output = P::Vector>,
{
    let l = left.get_far_point(direction, left_transform);
    let r = right.get_far_point(&direction.neg(), right_transform);
    SupportPoint {
        v: l - r,
        sup_a: l,
        sup_b: r,
    }
}
/*
#[cfg(test)]
mod tests {
    use cgmath::{Vector2, Rotation2, Rad, Point2, Point3, Quaternion, Rotation3};

    use super::{gjk, support, RunningAverage};
    use super::simplex::{SimplexProcessor2D, SimplexProcessor, SimplexProcessor3D};
    use Real;
    use collide::narrow::NarrowPhase;
    use collide2d::*;
    use collide3d::*;

    fn transform(x: Real, y: Real, angle: Real) -> BodyPose2D {
        BodyPose2D::new(Point2::new(x, y), Rotation2::from_angle(Rad(angle)))
    }

    fn transform_3d(x: Real, y: Real, z: Real, angle_z: Real) -> BodyPose3D {
        BodyPose3D::new(Point3::new(x, y, z), Quaternion::from_angle_z(Rad(angle_z)))
    }

    #[test]
    fn test_support() {
        let left = CollisionPrimitive2D::new(Rectangle::new(10., 10.).into());
        let left_transform = transform(15., 0., 0.);
        let right = CollisionPrimitive2D::new(Rectangle::new(10., 10.).into());
        let right_transform = transform(-15., 0., 0.);
        let direction = Vector2::new(1., 0.);
        assert_eq!(
            Vector2::new(40., 0.),
            support(&left, &left_transform, &right, &right_transform, &direction).v
        );
    }

    #[test]
    fn test_gjk_miss() {
        let left = CollisionPrimitive2D::new(Rectangle::new(10., 10.).into());
        let left_transform = transform(15., 0., 0.);
        let right = CollisionPrimitive2D::new(Rectangle::new(10., 10.).into());
        let right_transform = transform(-15., 0., 0.);
        let processor = SimplexProcessor2D::new();
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
        let left = CollisionPrimitive2D::new(Rectangle::new(10., 10.).into());
        let left_transform = transform(15., 0., 0.);
        let right = CollisionPrimitive2D::new(Rectangle::new(10., 10.).into());
        let right_transform = transform(7., 2., 0.);
        let processor = SimplexProcessor2D::new();
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
        let left = CollisionPrimitive3D::new(Box::new(10., 10., 10.).into());
        let left_transform = transform_3d(15., 0., 0., 0.);
        let right = CollisionPrimitive3D::new(Box::new(10., 10., 10.).into());
        let right_transform = transform_3d(7., 2., 0., 0.);
        let processor = SimplexProcessor3D::new();
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
        let left = CollisionShape2D::new_simple(
            CollisionStrategy::CollisionOnly,
            Rectangle::new(10., 10.).into(),
        );
        let left_transform = transform(15., 0., 0.);
        let right = CollisionShape2D::new_simple(
            CollisionStrategy::CollisionOnly,
            Rectangle::new(10., 10.).into(),
        );
        let right_transform = transform(-15., 0., 0.);
        let mut gjk = GJK2D::new();
        assert!(
            gjk.collide((1, &left, &left_transform), (2, &right, &right_transform))
                .is_none()
        );
    }

    #[test]
    fn test_gjk_shape_simple_hit() {
        let left = CollisionShape2D::new_simple(
            CollisionStrategy::CollisionOnly,
            Rectangle::new(10., 10.).into(),
        );
        let left_transform = transform(15., 0., 0.);
        let right = CollisionShape2D::new_simple(
            CollisionStrategy::CollisionOnly,
            Rectangle::new(10., 10.).into(),
        );
        let right_transform = transform(7., 2., 0.);
        let mut gjk = GJK2D::new();
        let set = gjk.collide((1, &left, &left_transform), (2, &right, &right_transform));
        assert!(set.is_some());
        let contact_set = set.unwrap();
        assert_eq!((1, 2), contact_set.bodies);
        assert_eq!(1, contact_set.contacts.len());
    }

    #[test]
    fn test_gjk_3d_shape_hit() {
        let left = CollisionShape3D::new_simple(
            CollisionStrategy::CollisionOnly,
            Box::new(10., 10., 10.).into(),
        );
        let left_transform = transform_3d(15., 0., 0., 0.);
        let right = CollisionShape3D::new_simple(
            CollisionStrategy::CollisionOnly,
            Box::new(10., 10., 10.).into(),
        );
        let right_transform = transform_3d(7., 2., 0., 0.);
        let mut gjk = GJK3D::new();
        let set = gjk.collide((1, &left, &left_transform), (2, &right, &right_transform));
        assert!(set.is_some());
        let contact_set = set.unwrap();
        assert_eq!((1, 2), contact_set.bodies);
        assert_eq!(1, contact_set.contacts.len());
    }

    #[test]
    fn test_gjk_shape_complex_miss() {
        let left = CollisionShape2D::new_complex(
            CollisionStrategy::CollisionOnly,
            vec![
                CollisionPrimitive2D::new_impl(
                    Rectangle::new(10., 10.).into(),
                    transform(0., 5., 0.)
                ),
                CollisionPrimitive2D::new_impl(
                    Rectangle::new(10., 10.).into(),
                    transform(0., -5., 0.)
                ),
            ],
        );
        let left_transform = transform(15., 0., 0.);
        let right = CollisionShape2D::new_simple(
            CollisionStrategy::CollisionOnly,
            Rectangle::new(10., 10.).into(),
        );
        let right_transform = transform(-15., 0., 0.);
        let mut gjk = GJK2D::new();
        assert!(
            gjk.collide((1, &left, &left_transform), (2, &right, &right_transform))
                .is_none()
        );
    }

    #[test]
    fn test_gjk_shape_complex_hit_single() {
        let left = CollisionShape2D::new_complex(
            CollisionStrategy::CollisionOnly,
            vec![
                CollisionPrimitive2D::new_impl(
                    Rectangle::new(10., 10.).into(),
                    transform(0., 5., 0.)
                ),
                CollisionPrimitive2D::new_impl(
                    Rectangle::new(10., 10.).into(),
                    transform(0., -5., 0.)
                ),
            ],
        );
        let left_transform = transform(15., 0., 0.);
        let right = CollisionShape2D::new_simple(
            CollisionStrategy::CollisionOnly,
            Rectangle::new(10., 10.).into(),
        );
        let right_transform = transform(7., 6., 0.);
        let mut gjk = GJK2D::new();
        let set = gjk.collide((1, &left, &left_transform), (2, &right, &right_transform));
        assert!(set.is_some());
        let contact_set = set.unwrap();
        assert_eq!((1, 2), contact_set.bodies);
        assert_eq!(1, contact_set.contacts.len());
    }

    #[test]
    fn test_gjk_shape_complex_hit_both() {
        let left = CollisionShape2D::new_complex(
            CollisionStrategy::CollisionOnly,
            vec![
                CollisionPrimitive2D::new_impl(
                    Rectangle::new(10., 10.).into(),
                    transform(0., 5., 0.)
                ),
                CollisionPrimitive2D::new_impl(
                    Rectangle::new(10., 10.).into(),
                    transform(0., -5., 0.)
                ),
            ],
        );
        let left_transform = transform(15., 0., 0.);
        let right = CollisionShape2D::new_simple(
            CollisionStrategy::CollisionOnly,
            Rectangle::new(10., 10.).into(),
        );
        let right_transform = transform(7., 4., 0.);
        let mut gjk = GJK2D::new();
        let set = gjk.collide((1, &left, &left_transform), (2, &right, &right_transform));
        assert!(set.is_some());
        let contact_set = set.unwrap();
        assert_eq!((1, 2), contact_set.bodies);
        assert_eq!(2, contact_set.contacts.len());
    }
}
*/
