pub mod simplex;
pub mod epa;

use super::NarrowPhase;

use collide::{Contact, ContactSet, CollisionShape, CollisionStrategy, CollisionPrimitive};

use cgmath::prelude::*;
use cgmath::{BaseFloat, Decomposed};
use collision::{Aabb, Discrete, MinMax};

use std;
use std::ops::Neg;
use std::fmt::Debug;

use self::simplex::SimplexProcessor;

const MAX_ITERATIONS: u32 = 100;

#[derive(Debug)]
struct RunningAverage {
    pub average : f64,
    n : u32
}

impl RunningAverage {
    pub fn new() -> Self {
        Self {
            average : 0.,
            n : 0
        }
    }

    pub fn add(&mut self, value: u32) {
        let n0 = self.n as f64;
        self.n += 1;
        let n = self.n as f64;
        self.average *= n0 / n;
        self.average += (value as f64) / n;
    }
}

pub struct GJK<S, V, P>
where
    S: BaseFloat,
    V: VectorSpace<Scalar = S> + ElementWise + Array<Element = S> + InnerSpace,
    P: SimplexProcessor<S, V>,
{
    simplex_processor: P,
    average: RunningAverage,
    marker: std::marker::PhantomData<(S, V)>,
}

impl<S, V, P> GJK<S, V, P>
where
    S: BaseFloat,
    V: VectorSpace<Scalar = S> + ElementWise + Array<Element = S> + InnerSpace,
    P: SimplexProcessor<S, V>,
{
    pub fn new() -> Self {
        Self {
            simplex_processor: P::new(),
            average: RunningAverage::new(),
            marker: std::marker::PhantomData,
        }
    }

    pub fn get_average(&self) -> f64 {
        self.average.average
    }
}

impl<ID, S, V, P, A, R, SP> NarrowPhase<ID, S, V, P, R, A> for GJK<S, V, SP>
where
    ID: Clone + Debug,
    S: BaseFloat,
    V: VectorSpace<Scalar = S>
        + ElementWise
        + Array<Element = S>
        + MetricSpace
        + InnerSpace
        + Neg<Output = V>,
    P: EuclideanSpace<
        Scalar = S,
        Diff = V,
    >
        + MinMax + Debug,
    A: Aabb<S, V, P> + Discrete<A> + Clone,
    R: Rotation<P>,
    SP: SimplexProcessor<S, V>
{
    fn collide(
        &mut self,
        (ref left_id, ref left, ref left_transform):
            (ID, &CollisionShape<S, V, P, R, A>, &Decomposed<V, R>),
        (ref right_id, ref right, ref right_transform):
            (ID, &CollisionShape<S, V, P, R, A>, &Decomposed<V, R>),
    ) -> Option<ContactSet<ID, S, V>> {
        let mut contacts = Vec::default();
        if !left.enabled || !right.enabled || left.primitives.is_empty() ||
            right.primitives.is_empty()
        {
            return None;
        }

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
                                contacts.append(&mut self::epa::epa(&mut simplex,
                                                              &left_primitive,
                                                                    left_transform,
                                                             &right_primitive,
                                                             right_transform,
                                                            &self.simplex_processor));
                            }
                        }
                        None => (),
                    };
                }
            }
        }

        if contacts.len() > 0 {
            Some(ContactSet::new((left_id.clone(), right_id.clone()), contacts))
        } else {
            None
        }
    }
}

fn gjk<S, V, P, A, R, SP>(
    left: &CollisionPrimitive<S, V, P, R, A>,
    left_transform: &Decomposed<V, R>,
    right: &CollisionPrimitive<S, V, P, R, A>,
    right_transform: &Decomposed<V, R>,
    simplex_processor: &SP,
    average: &mut RunningAverage,
) -> Option<Vec<V>>
where
    S: BaseFloat,
    V: VectorSpace<Scalar = S> + ElementWise + Array<Element = S> + InnerSpace + Neg<Output = V>,
    P: EuclideanSpace<Scalar = S, Diff = V> + MinMax + Debug,
    A: Aabb<S, V, P> + Discrete<A> + Clone,
    R: Rotation<P>,
    SP: SimplexProcessor<S, V>,
{
    let mut d = right_transform.disp - left_transform.disp;
    let a = support(left, left_transform, right, right_transform, &d);
    if a.dot(d) <= S::zero() {
        average.add(0);
        return None;
    }
    let mut simplex: Vec<V> = Vec::default();
    simplex.push(a);
    d = d.neg();
    let mut i = 0;
    loop {
        let a = support(left, left_transform, right, right_transform, &d);
        if a.dot(d) <= S::zero() {
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

pub fn support<S, V, P, A, R>(
    left: &CollisionPrimitive<S, V, P, R, A>,
    left_transform: &Decomposed<V, R>,
    right: &CollisionPrimitive<S, V, P, R, A>,
    right_transform: &Decomposed<V, R>,
    direction: &V,
) -> V
where
    S: BaseFloat,
    V: VectorSpace<Scalar = S> + ElementWise + Array<Element = S> + Neg<Output = V>,
    P: EuclideanSpace<Scalar = S, Diff = V> + MinMax + Debug,
    A: Aabb<S, V, P> + Discrete<A> + Clone,
    R: Rotation<P>,
{
    let l = left.get_far_point(direction, left_transform);
    let r = right.get_far_point(&direction.neg(), right_transform);
    l - r
}

#[cfg(test)]
mod tests {
    use super::{gjk, support, RunningAverage};
    use super::simplex::{SimplexProcessor2D, SimplexProcessor};
    use cgmath::{Vector2, Decomposed, Rotation2, Rad, Basis2};
    use collide2d::*;
    use collide::narrow::NarrowPhase;

    #[test]
    fn test_support() {
        let left = CollisionPrimitive2D::<f32>::new(Rectangle::new(10., 10.));
        let left_transform = Decomposed::<Vector2<f32>, Basis2<f32>> {
            disp: Vector2::new(15., 0.),
            rot: Rotation2::from_angle(Rad(0.)),
            scale: 1.,
        };
        let right = CollisionPrimitive2D::<f32>::new(Rectangle::new(10., 10.));
        let right_transform = Decomposed::<Vector2<f32>, Basis2<f32>> {
            disp: Vector2::new(-15., 0.),
            rot: Rotation2::from_angle(Rad(0.)),
            scale: 1.,
        };
        let direction = Vector2::new(1., 0.);
        assert_eq!(
            Vector2::new(40., 0.),
            support(&left, &left_transform, &right, &right_transform, &direction)
        );
    }

    #[test]
    fn test_gjk_miss() {
        let left = CollisionPrimitive2D::<f32>::new(Rectangle::new(10., 10.));
        let left_transform = Decomposed::<Vector2<f32>, Basis2<f32>> {
            disp: Vector2::new(15., 0.),
            rot: Rotation2::from_angle(Rad(0.)),
            scale: 1.,
        };
        let right = CollisionPrimitive2D::<f32>::new(Rectangle::new(10., 10.));
        let right_transform = Decomposed::<Vector2<f32>, Basis2<f32>> {
            disp: Vector2::new(-15., 0.),
            rot: Rotation2::from_angle(Rad(0.)),
            scale: 1.,
        };
        let processor =
            <SimplexProcessor2D as SimplexProcessor<f32, Vector2<f32>>>::new();
        let mut average = RunningAverage::new();
        assert_eq!(
            None,
            gjk(&left, &left_transform, &right, &right_transform, &processor, &mut average)
        );
    }

    #[test]
    fn test_gjk_hit() {
        let left = CollisionPrimitive2D::<f32>::new(Rectangle::new(10., 10.));
        let left_transform = Decomposed::<Vector2<f32>, Basis2<f32>> {
            disp: Vector2::new(15., 0.),
            rot: Rotation2::from_angle(Rad(0.)),
            scale: 1.,
        };
        let right = CollisionPrimitive2D::<f32>::new(Rectangle::new(10., 10.));
        let right_transform = Decomposed::<Vector2<f32>, Basis2<f32>> {
            disp: Vector2::new(7., 2.),
            rot: Rotation2::from_angle(Rad(0.)),
            scale: 1.,
        };
        let processor =
            <SimplexProcessor2D as SimplexProcessor<f32, Vector2<f32>>>::new();
        let mut average = RunningAverage::new();
        assert!(gjk(&left, &left_transform, &right, &right_transform, &processor,
                    &mut average).is_some());
    }

    #[test]
    fn test_gjk_shape_simple_miss() {
        let left = CollisionShape2D::<f32>::new_simple(
            CollisionStrategy::CollisionOnly,
            Rectangle::new(10., 10.),
        );
        let left_transform = Decomposed::<Vector2<f32>, Basis2<f32>> {
            disp: Vector2::new(15., 0.),
            rot: Rotation2::from_angle(Rad(0.)),
            scale: 1.,
        };
        let right = CollisionShape2D::<f32>::new_simple(
            CollisionStrategy::CollisionOnly,
            Rectangle::new(10., 10.),
        );
        let right_transform = Decomposed::<Vector2<f32>, Basis2<f32>> {
            disp: Vector2::new(-15., 0.),
            rot: Rotation2::from_angle(Rad(0.)),
            scale: 1.,
        };
        let mut gjk = GJK2D::new();
        assert!(
            gjk.collide((1, &left, &left_transform), (2, &right, &right_transform))
                .is_none()
        );
    }

    #[test]
    fn test_gjk_shape_simple_hit() {
        let left = CollisionShape2D::<f32>::new_simple(
            CollisionStrategy::CollisionOnly,
            Rectangle::new(10., 10.),
        );
        let left_transform = Decomposed::<Vector2<f32>, Basis2<f32>> {
            disp: Vector2::new(15., 0.),
            rot: Rotation2::from_angle(Rad(0.)),
            scale: 1.,
        };
        let right = CollisionShape2D::<f32>::new_simple(
            CollisionStrategy::CollisionOnly,
            Rectangle::new(10., 10.),
        );
        let right_transform = Decomposed::<Vector2<f32>, Basis2<f32>> {
            disp: Vector2::new(7., 2.),
            rot: Rotation2::from_angle(Rad(0.)),
            scale: 1.,
        };
        let mut gjk = GJK2D::new();
        let set = gjk.collide((1, &left, &left_transform), (2, &right, &right_transform));
        assert!(set.is_some());
        let contact_set = set.unwrap();
        assert_eq!((1, 2), contact_set.bodies);
        assert_eq!(1, contact_set.contacts.len());
    }

    #[test]
    fn test_gjk_shape_complex_miss() {
        let left = CollisionShape2D::<f32>::new_complex(
            CollisionStrategy::CollisionOnly,
            vec![
                CollisionPrimitive2D::new_impl(
                    Rectangle::new(10., 10.),
                    Decomposed::<Vector2<f32>, Basis2<f32>> {
                        disp: Vector2::new(0., 5.),
                        scale: 1.,
                        rot: Rotation2::from_angle(Rad(0.)),
                    }
                ),
                CollisionPrimitive2D::new_impl(
                    Rectangle::new(10., 10.),
                    Decomposed::<Vector2<f32>, Basis2<f32>> {
                        disp: Vector2::new(0., -5.),
                        scale: 1.,
                        rot: Rotation2::from_angle(Rad(0.)),
                    }
                ),
            ],
        );
        let left_transform = Decomposed::<Vector2<f32>, Basis2<f32>> {
            disp: Vector2::new(15., 0.),
            rot: Rotation2::from_angle(Rad(0.)),
            scale: 1.,
        };
        let right = CollisionShape2D::<f32>::new_simple(
            CollisionStrategy::CollisionOnly,
            Rectangle::new(10., 10.),
        );
        let right_transform = Decomposed::<Vector2<f32>, Basis2<f32>> {
            disp: Vector2::new(-15., 0.),
            rot: Rotation2::from_angle(Rad(0.)),
            scale: 1.,
        };
        let mut gjk = GJK2D::new();
        assert!(
            gjk.collide((1, &left, &left_transform), (2, &right, &right_transform))
                .is_none()
        );
    }

    #[test]
    fn test_gjk_shape_complex_hit_single() {
        let left = CollisionShape2D::<f32>::new_complex(
            CollisionStrategy::CollisionOnly,
            vec![
                CollisionPrimitive2D::new_impl(
                    Rectangle::new(10., 10.),
                    Decomposed::<Vector2<f32>, Basis2<f32>> {
                        disp: Vector2::new(0., 5.),
                        scale: 1.,
                        rot: Rotation2::from_angle(Rad(0.)),
                    }
                ),
                CollisionPrimitive2D::new_impl(
                    Rectangle::new(10., 10.),
                    Decomposed::<Vector2<f32>, Basis2<f32>> {
                        disp: Vector2::new(0., -5.),
                        scale: 1.,
                        rot: Rotation2::from_angle(Rad(0.)),
                    }
                ),
            ],
        );
        let left_transform = Decomposed::<Vector2<f32>, Basis2<f32>> {
            disp: Vector2::new(15., 0.),
            rot: Rotation2::from_angle(Rad(0.)),
            scale: 1.,
        };
        let right = CollisionShape2D::<f32>::new_simple(
            CollisionStrategy::CollisionOnly,
            Rectangle::new(10., 10.),
        );
        let right_transform = Decomposed::<Vector2<f32>, Basis2<f32>> {
            disp: Vector2::new(7., 6.),
            rot: Rotation2::from_angle(Rad(0.)),
            scale: 1.,
        };
        let mut gjk = GJK2D::new();
        let set = gjk.collide((1, &left, &left_transform), (2, &right, &right_transform));
        assert!(set.is_some());
        let contact_set = set.unwrap();
        assert_eq!((1, 2), contact_set.bodies);
        assert_eq!(1, contact_set.contacts.len());
    }

    #[test]
    fn test_gjk_shape_complex_hit_both() {
        let left = CollisionShape2D::<f32>::new_complex(
            CollisionStrategy::CollisionOnly,
            vec![
                CollisionPrimitive2D::new_impl(
                    Rectangle::new(10., 10.),
                    Decomposed::<Vector2<f32>, Basis2<f32>> {
                        disp: Vector2::new(0., 5.),
                        scale: 1.,
                        rot: Rotation2::from_angle(Rad(0.)),
                    }
                ),
                CollisionPrimitive2D::new_impl(
                    Rectangle::new(10., 10.),
                    Decomposed::<Vector2<f32>, Basis2<f32>> {
                        disp: Vector2::new(0., -5.),
                        scale: 1.,
                        rot: Rotation2::from_angle(Rad(0.)),
                    }
                ),
            ],
        );
        let left_transform = Decomposed::<Vector2<f32>, Basis2<f32>> {
            disp: Vector2::new(15., 0.),
            rot: Rotation2::from_angle(Rad(0.)),
            scale: 1.,
        };
        let right = CollisionShape2D::<f32>::new_simple(
            CollisionStrategy::CollisionOnly,
            Rectangle::new(10., 10.),
        );
        let right_transform = Decomposed::<Vector2<f32>, Basis2<f32>> {
            disp: Vector2::new(7., 4.),
            rot: Rotation2::from_angle(Rad(0.)),
            scale: 1.,
        };
        let mut gjk = GJK2D::new();
        let set = gjk.collide((1, &left, &left_transform), (2, &right, &right_transform));
        assert!(set.is_some());
        let contact_set = set.unwrap();
        assert_eq!((1, 2), contact_set.bodies);
        assert_eq!(2, contact_set.contacts.len());
    }
}
