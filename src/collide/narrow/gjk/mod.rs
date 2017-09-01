pub mod simplex;
pub mod epa;

use std;
use std::fmt::Debug;
use std::ops::Neg;

use cgmath::prelude::*;
use collision::{Aabb, Discrete};

use self::simplex::SimplexProcessor;
use super::NarrowPhase;
use {Pose, Real};
use collide::{Contact, ContactSet, CollisionShape, CollisionStrategy, CollisionPrimitive,
              Primitive};

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

pub struct GJK<V, S> {
    simplex_processor: S,
    average: RunningAverage,
    marker: std::marker::PhantomData<V>,
}

impl<V, S> GJK<V, S>
where
    V: VectorSpace<Scalar = Real>,
    S: SimplexProcessor<Vector = V>,
{
    pub fn new() -> Self {
        Self {
            simplex_processor: S::new(),
            average: RunningAverage::new(),
            marker: std::marker::PhantomData,
        }
    }

    pub fn get_average(&self) -> f64 {
        self.average.average
    }
}

impl<ID, P, A, T, S> NarrowPhase<ID, P, A, T> for GJK<A::Diff, S>
where
    ID: Debug + Clone,
    A: Aabb<Scalar=Real> + Discrete<A> + Clone,
    A::Diff: Debug
        + InnerSpace
        + Neg<Output = A::Diff>,
    S: SimplexProcessor<Vector = A::Diff>,
    P: Primitive<A>,
    T: Pose<A::Point>,
{
    fn collide(
        &mut self,
        (ref left_id, ref left, ref left_transform): (ID, &CollisionShape<P, A, T>, &T),
        (ref right_id, ref right, ref right_transform): (ID, &CollisionShape<P, A, T>, &T),
    ) -> Option<ContactSet<ID, A::Diff>> {
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
                                contacts.append(&mut self::epa::epa(
                                    &mut simplex,
                                    &left_primitive,
                                    left_transform,
                                    &right_primitive,
                                    right_transform,
                                    &self.simplex_processor,
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

fn gjk<P, A, T, S>(
    left: &CollisionPrimitive<P, A, T>,
    left_transform: &T,
    right: &CollisionPrimitive<P, A, T>,
    right_transform: &T,
    simplex_processor: &S,
    average: &mut RunningAverage,
) -> Option<Vec<A::Diff>>
where
    A: Aabb<Scalar = Real> + Clone,
    A::Diff: Debug + Neg<Output = A::Diff> + InnerSpace,
    P: Primitive<A>,
    T: Pose<A::Point>,
    S: SimplexProcessor<Vector = A::Diff>,
{
    let mut d = *right_transform.position() - *left_transform.position();
    let a = support(left, left_transform, right, right_transform, &d);
    if a.dot(d) <= 0. {
        average.add(0);
        return None;
    }
    let mut simplex: Vec<A::Diff> = Vec::default();
    simplex.push(a);
    d = d.neg();
    let mut i = 0;
    loop {
        let a = support(left, left_transform, right, right_transform, &d);
        if a.dot(d) <= 0. {
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

pub fn support<P, A, T>(
    left: &CollisionPrimitive<P, A, T>,
    left_transform: &T,
    right: &CollisionPrimitive<P, A, T>,
    right_transform: &T,
    direction: &A::Diff,
) -> A::Diff
where
    A: Aabb<Scalar = Real> + Clone,
    A::Diff: Neg<Output = A::Diff>,
    P: Primitive<A>,
    T: Pose<A::Point>,
{
    let l = left.get_far_point(direction, left_transform);
    let r = right.get_far_point(&direction.neg(), right_transform);
    l - r
}

#[cfg(test)]
mod tests {
    use cgmath::{Vector2, Rotation2, Rad, Point2};

    use super::{gjk, support, RunningAverage};
    use super::simplex::{SimplexProcessor2D, SimplexProcessor};
    use Real;
    use collide::narrow::NarrowPhase;
    use collide2d::*;

    fn transform(x: Real, y: Real, angle: Real) -> BodyPose2D {
        BodyPose2D::new(Point2::new(x, y), Rotation2::from_angle(Rad(angle)))
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
            support(&left, &left_transform, &right, &right_transform, &direction)
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
        assert_eq!(
            None,
            gjk(
                &left,
                &left_transform,
                &right,
                &right_transform,
                &processor,
                &mut average,
            )
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
        assert!(
            gjk(
                &left,
                &left_transform,
                &right,
                &right_transform,
                &processor,
                &mut average,
            ).is_some()
        );
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
