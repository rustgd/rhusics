//! Generic narrow phase collision detection algorithms.
//!
//! Currently only supports GJK/EPA.

use std::fmt::Debug;
use std::ops::Neg;

use cgmath::BaseFloat;
use cgmath::prelude::*;
use collision::{CollisionStrategy, Contact, Interpolate, Primitive};
use collision::algorithm::minkowski::{SimplexProcessor, EPA, GJK};
use collision::prelude::*;

use collide::{Collider, CollisionData, CollisionMode, CollisionShape, ContactEvent};

/// Base trait implemented by all narrow phase algorithms.
///
/// # Type parameters:
///
/// - `P`: collision primitive type
/// - `T`: model-to-world transform type
/// - `B`: Bounding volume
/// - `Y`: Shape type (see `Collider`)
pub trait NarrowPhase<P, T, B, Y = ()>: Send
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
        left: &CollisionShape<P, T, B, Y>,
        left_transform: &T,
        right: &CollisionShape<P, T, B, Y>,
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
        left: &CollisionShape<P, T, B, Y>,
        left_start_transform: &T,
        left_end_transform: Option<&T>,
        right: &CollisionShape<P, T, B, Y>,
        right_start_transform: &T,
        right_end_transform: Option<&T>,
    ) -> Option<Contact<P::Point>>;
}

impl<P, T, Y, S, E, B> NarrowPhase<P, T, B, Y> for GJK<S, E, <P::Point as EuclideanSpace>::Scalar>
where
    P: Primitive,
    P::Point: EuclideanSpace,
    <P::Point as EuclideanSpace>::Scalar: BaseFloat + Send + Sync + 'static,
    <P::Point as EuclideanSpace>::Diff: Debug
        + InnerSpace
        + Array<Element = <P::Point as EuclideanSpace>::Scalar>
        + Neg<Output = <P::Point as EuclideanSpace>::Diff>,
    S: SimplexProcessor<Point = P::Point> + Send,
    E: EPA<Point = P::Point> + Send,
    T: Transform<P::Point>
        + Interpolate<<P::Point as EuclideanSpace>::Scalar>
        + TranslationInterpolate<<P::Point as EuclideanSpace>::Scalar>,
    Y: Collider,
{
    fn collide(
        &self,
        left: &CollisionShape<P, T, B, Y>,
        left_transform: &T,
        right: &CollisionShape<P, T, B, Y>,
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
        left: &CollisionShape<P, T, B, Y>,
        left_start_transform: &T,
        left_end_transform: Option<&T>,
        right: &CollisionShape<P, T, B, Y>,
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

/// Perform narrow phase collision detection on the given potential collider pairs, using the given
/// narrow phase
///
/// ### Type parameters:
///
/// - `C`: Collision data
/// - `I`: Id, returned by `GetId` on `D`, primary id for a collider
/// - `P`: Primitive
/// - `T`: Transform
/// - `B`: Bounding volume, not used here, but required for `CollisionData`
/// - `Y`: Collider, see `Collider` for more information, not used here, but required for
///        `CollisionData`
/// - `D`: Broad phase data, not used here, but required for `CollisionData`
pub fn narrow_collide<C, I, P, T, B, Y, D>(
    data: &C,
    narrow: &Box<NarrowPhase<P, T, B, Y>>,
    potentials: Vec<(I, I)>,
) -> Vec<ContactEvent<I, P::Point>>
where
    C: CollisionData<I, P, T, B, Y, D>,
    P: Primitive,
    <P::Point as EuclideanSpace>::Diff: Debug,
    I: Copy + Debug,
{
    potentials
        .iter()
        .filter_map(|&(left, right)| {
            let left_shape = data.get_shape(left);
            let right_shape = data.get_shape(right);
            let left_pose = data.get_pose(left);
            let right_pose = data.get_pose(right);
            let left_next_pose = data.get_next_pose(left);
            let right_next_pose = data.get_next_pose(right);
            narrow
                .collide_continuous(
                    left_shape,
                    left_pose,
                    left_next_pose,
                    right_shape,
                    right_pose,
                    right_next_pose,
                )
                .map(|contact| ContactEvent::new((left, right), contact))
        })
        .collect::<Vec<_>>()
}

#[cfg(test)]
mod tests {

    use cgmath::{BaseFloat, Basis2, Decomposed, Rad, Rotation2, Vector2};
    use collision::Aabb2;
    use collision::algorithm::minkowski::GJK2;
    use collision::primitive::Rectangle;

    use collide::*;
    use collide::narrow::NarrowPhase;

    fn transform<S>(x: S, y: S, angle: S) -> Decomposed<Vector2<S>, Basis2<S>>
    where
        S: BaseFloat,
    {
        Decomposed {
            disp: Vector2::new(x, y),
            rot: Rotation2::from_angle(Rad(angle)),
            scale: S::one(),
        }
    }

    #[test]
    fn test_gjk_continuous_2d_f32() {
        let left = CollisionShape::<_, _, Aabb2<_>, ()>::new_simple(
            CollisionStrategy::FullResolution,
            CollisionMode::Continuous,
            Rectangle::new(10., 10.),
        );
        let left_start_transform = transform::<f32>(0., 0., 0.);
        let left_end_transform = transform(30., 0., 0.);
        let right = CollisionShape::new_simple(
            CollisionStrategy::FullResolution,
            CollisionMode::Discrete,
            Rectangle::new(10., 10.),
        );
        let right_transform = transform(15., 0., 0.);
        let gjk = GJK2::<f32>::new();

        assert!(gjk.collide_continuous(
            &left,
            &left_start_transform,
            Some(&left_start_transform),
            &right,
            &right_transform,
            Some(&right_transform)
        ).is_none());

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

    #[test]
    fn test_gjk_continuous_2d_f64() {
        let left = CollisionShape::<_, _, Aabb2<_>, ()>::new_simple(
            CollisionStrategy::FullResolution,
            CollisionMode::Continuous,
            Rectangle::new(10., 10.),
        );
        let left_start_transform = transform::<f64>(0., 0., 0.);
        let left_end_transform = transform(30., 0., 0.);
        let right = CollisionShape::new_simple(
            CollisionStrategy::FullResolution,
            CollisionMode::Discrete,
            Rectangle::new(10., 10.),
        );
        let right_transform = transform(15., 0., 0.);
        let gjk = GJK2::<f64>::new();

        assert!(gjk.collide_continuous(
            &left,
            &left_start_transform,
            Some(&left_start_transform),
            &right,
            &right_transform,
            Some(&right_transform)
        ).is_none());

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
