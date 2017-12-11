//! Generic narrow phase collision detection algorithms.
//!
//! Currently only supports GJK/EPA.

use std::fmt::Debug;
use std::ops::Neg;

use cgmath::prelude::*;
use collision::{CollisionStrategy, Contact, Interpolate, Primitive};
use collision::algorithm::minkowski::{SimplexProcessor, EPA, GJK};
use collision::prelude::*;

use Real;
use collide::{Collider, CollisionMode, CollisionShape};

/// Base trait implemented by all narrow phase algorithms.
///
/// # Type parameters:
///
/// - `P`: collision primitive type
/// - `T`: model-to-world transform type
pub trait NarrowPhase<P, T, Y = ()>: Send
where
    P: Primitive,
    <P::Point as EuclideanSpace>::Diff: Debug,
{
    /// Check if two shapes collides, and give a contact manifold for the contact with the largest
    /// penetration depth.
    ///
    /// # Parameters:
    ///
    /// - `left`: the left shape
    /// - `left_transform`: model-to-world transform for the left shape
    /// - `right`: the right shape
    /// - `right_transform`: model-to-world transform for the right shape
    ///
    /// # Returns:
    ///
    /// Optionally returns the contact manifold for the contact with largest penetration depth
    fn collide(
        &self,
        left: &CollisionShape<P, T, Y>,
        left_transform: &T,
        right: &CollisionShape<P, T, Y>,
        right_transform: &T,
    ) -> Option<Contact<P::Point>>;

    /// Check if two shapes collides along the given transformation paths, and give a contact
    /// manifold for the contact with the earliest time of impact.
    ///
    /// Will only use continuous detection if one of the shapes have `Continuous` collision mode.
    ///
    ///
    /// # Parameters:
    ///
    /// - `left`: the left shape
    /// - `left_start_transform`: model-to-world transform for the left shape, at start of frame
    /// - `left_end_transform`: model-to-world transform for the left shape, at end of frame
    /// - `right`: the right shape
    /// - `right_start_transform`: model-to-world transform for the right shape, at start of frame
    /// - `right_end_transform`: model-to-world transform for the right shape, at end of frame
    ///
    /// # Returns:
    ///
    /// Optionally returns the contact manifold for the contact with largest penetration depth
    fn collide_continuous(
        &self,
        left: &CollisionShape<P, T, Y>,
        left_start_transform: &T,
        left_end_transform: Option<&T>,
        right: &CollisionShape<P, T, Y>,
        right_start_transform: &T,
        right_end_transform: Option<&T>,
    ) -> Option<Contact<P::Point>>;
}

impl<P, T, Y, S, E> NarrowPhase<P, T, Y> for GJK<S, E>
where
    P: Primitive,
    <P::Point as EuclideanSpace>::Diff: Debug
        + InnerSpace
        + Neg<Output = <P::Point as EuclideanSpace>::Diff>,
    P::Aabb: Discrete<P::Aabb> + Aabb<Scalar = Real>,
    S: SimplexProcessor<Point = P::Point> + Send,
    E: EPA<Point = P::Point> + Send,
    T: Transform<P::Point>
        + Interpolate<<P::Point as EuclideanSpace>::Scalar>
        + TranslationInterpolate<<P::Point as EuclideanSpace>::Scalar>,
    Y: Collider,
{
    fn collide(
        &self,
        left: &CollisionShape<P, T, Y>,
        left_transform: &T,
        right: &CollisionShape<P, T, Y>,
        right_transform: &T,
    ) -> Option<Contact<P::Point>> {
        if !left.enabled || !right.enabled || left.primitives.is_empty()
            || right.primitives.is_empty()
            || !left.ty.should_generate_contacts(&right.ty)
        {
            return None;
        }

        let strategy = max(&left.strategy, &right.strategy);
        self.intersection_complex(
            &strategy,
            &left.primitives,
            left_transform,
            &right.primitives,
            right_transform,
        )
    }

    fn collide_continuous(
        &self,
        left: &CollisionShape<P, T, Y>,
        left_start_transform: &T,
        left_end_transform: Option<&T>,
        right: &CollisionShape<P, T, Y>,
        right_start_transform: &T,
        right_end_transform: Option<&T>,
    ) -> Option<Contact<P::Point>> {
        if !left.ty.should_generate_contacts(&right.ty) {
            return None;
        }

        // fallback to start transforms if end transforms are not available
        let left_end_transform = match left_end_transform {
            Some(t) => t,
            None => left_start_transform,
        };
        let right_end_transform = match right_end_transform {
            Some(t) => t,
            None => right_start_transform,
        };

        if left.mode == CollisionMode::Continuous || right.mode == CollisionMode::Continuous {
            let strategy = max(&left.strategy, &right.strategy);
            // if the start of the transformation path has collision, return that contact
            self.collide(left, left_start_transform, right, right_start_transform)
                .or_else(|| {
                    // do time of impact calculation
                    self.intersection_complex_time_of_impact(
                        &strategy,
                        &left.primitives,
                        left_start_transform..left_end_transform,
                        &right.primitives,
                        right_start_transform..right_end_transform,
                    )
                })
        } else {
            self.collide(left, left_end_transform, right, right_end_transform)
        }
    }
}

fn max(left: &CollisionStrategy, right: &CollisionStrategy) -> CollisionStrategy {
    if left > right {
        left.clone()
    } else {
        right.clone()
    }
}

#[cfg(test)]
mod tests {

    use cgmath::{Basis2, Decomposed, Rad, Rotation2, Vector2};
    use collision::algorithm::minkowski::GJK2;
    use collision::primitive::Rectangle;

    use Real;
    use collide::*;
    use collide::narrow::NarrowPhase;

    fn transform(x: Real, y: Real, angle: Real) -> Decomposed<Vector2<Real>, Basis2<Real>> {
        Decomposed {
            disp: Vector2::new(x, y),
            rot: Rotation2::from_angle(Rad(angle)),
            scale: 1.,
        }
    }

    #[test]
    fn test_gjk_continuous_2d() {
        let left = CollisionShape::<_, _, ()>::new_simple(
            CollisionStrategy::FullResolution,
            CollisionMode::Continuous,
            Rectangle::new(10., 10.),
        );
        let left_start_transform = transform(0., 0., 0.);
        let left_end_transform = transform(30., 0., 0.);
        let right = CollisionShape::new_simple(
            CollisionStrategy::FullResolution,
            CollisionMode::Discrete,
            Rectangle::new(10., 10.),
        );
        let right_transform = transform(15., 0., 0.);
        let gjk = GJK2::<Real>::new();

        assert!(
            gjk.collide_continuous(
                &left,
                &left_start_transform,
                Some(&left_start_transform),
                &right,
                &right_transform,
                Some(&right_transform)
            ).is_none()
        );

        let contact = gjk.collide_continuous(
            &left,
            &left_start_transform,
            Some(&left_end_transform),
            &right,
            &right_transform,
            Some(&right_transform),
        ).unwrap();

        assert_ulps_eq!(0.16666666666666666, contact.time_of_impact);

        println!("{:?}", contact);
    }
}
