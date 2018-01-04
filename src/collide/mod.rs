//! Generic data structures and algorithms for collision detection

pub use collision::{CollisionStrategy, ComputeBound, Contact};
pub use collision::prelude::Primitive;

pub mod narrow;
pub mod broad;
pub mod prelude2d;
pub mod prelude3d;

use std::fmt::Debug;

use cgmath::prelude::*;
use collision::prelude::*;

/// Used to check if two shapes should be checked for collisions
pub trait Collider {
    /// Should shapes generate contact events
    fn should_generate_contacts(&self, other: &Self) -> bool;
}

impl<'a> Collider for () {
    fn should_generate_contacts(&self, _: &Self) -> bool {
        true
    }
}

/// Control continuous mode for shapes
#[derive(Debug, Clone, PartialOrd, PartialEq)]
pub enum CollisionMode {
    /// Discrete collision mode
    Discrete,

    /// Continuous collision mode
    Continuous,
}

/// Contains all contact information for a single contact, together with IDs of the colliding bodies
///
/// # Type parameters
///
/// - `ID`: The ID type of the body. This is supplied by the user of the library. In the ECS case,
///         this will be [`Entity`](https://docs.rs/specs/0.9.5/specs/struct.Entity.html).
/// - `V`: cgmath vector type
#[derive(Debug, Clone)]
pub struct ContactEvent<ID, P>
where
    P: EuclideanSpace,
    P::Diff: Debug,
{
    /// The ids of the two colliding bodies
    pub bodies: (ID, ID),

    /// The contact between the colliding bodies
    pub contact: Contact<P>,
}

impl<ID, P> ContactEvent<ID, P>
where
    ID: Clone + Debug,
    P: EuclideanSpace,
    P::Diff: VectorSpace + Zero + Debug,
{
    /// Create a new contact event
    pub fn new(bodies: (ID, ID), contact: Contact<P>) -> Self {
        Self { bodies, contact }
    }

    /// Convenience function to create a contact set with a simple [`Contact`](struct.Contact.html).
    pub fn new_simple(strategy: CollisionStrategy, bodies: (ID, ID)) -> Self {
        Self::new(bodies, Contact::new(strategy))
    }
}

/// Collision shape describing a complete collision object in the collision world.
///
/// Can handle both convex shapes, and concave shapes, by subdividing the concave shapes into
/// multiple convex shapes. This task is up to the user of the library to perform, no subdivision is
/// done automatically in the library.
///
/// Contains cached information about the base bounding box containing all primitives,
/// in model space coordinates. Also contains a cached version of the transformed bounding box,
/// in world space coordinates.
///
/// Also have details about what collision strategy/mode to use for contact resolution with this
/// shape.
///
/// ### Type parameters:
///
/// - `P`: Primitive type
/// - `T`: Transform type
/// - `B`: Bounding volume type
/// - `Y`: Shape type (see `Collider`)
#[derive(Debug, Clone)]
pub struct CollisionShape<P, T, B, Y = ()>
where
    P: Primitive,
{
    /// Enable/Disable collision detection for this shape
    pub enabled: bool,
    base_bound: B,
    transformed_bound: B,
    primitives: Vec<(P, T)>,
    strategy: CollisionStrategy,
    mode: CollisionMode,
    ty: Y,
}

impl<P, T, B, Y> CollisionShape<P, T, B, Y>
where
    P: Primitive + ComputeBound<B>,
    B: Bound<Point = P::Point> + Union<B, Output = B> + Clone,
    T: Transform<P::Point>,
    Y: Default,
{
    /// Create a new collision shape, with multiple collision primitives.
    ///
    /// Will compute and cache the base bounding box that contains all the given primitives,
    /// in model space coordinates.
    ///
    /// # Parameters
    ///
    /// - `strategy`: The collision strategy to use for this shape.
    /// - `primitives`: List of all primitives that make up this shape.
    /// - `ty`: The shape type, use () if not needed
    pub fn new_complex(
        strategy: CollisionStrategy,
        mode: CollisionMode,
        primitives: Vec<(P, T)>,
        ty: Y,
    ) -> Self {
        let bound: B = get_bound(&primitives);
        Self {
            base_bound: bound.clone(),
            primitives,
            enabled: true,
            transformed_bound: bound,
            strategy,
            mode,
            ty,
        }
    }

    /// Convenience function to create a simple collision shape with only a single given primitive,
    /// with no local-to-model transform.
    ///
    /// # Parameters
    ///
    /// - `strategy`: The collision strategy to use for this shape.
    /// - `primitive`: The collision primitive.
    pub fn new_simple(strategy: CollisionStrategy, mode: CollisionMode, primitive: P) -> Self {
        Self::new_complex(
            strategy,
            mode,
            vec![(primitive, T::one())],
            Default::default(),
        )
    }

    /// Convenience function to create a simple collision shape with only a single given primitive,
    /// and a shape type, with no local-to-model transform.
    ///
    /// # Parameters
    ///
    /// - `strategy`: The collision strategy to use for this shape.
    /// - `primitive`: The collision primitive.
    pub fn new_simple_with_type(
        strategy: CollisionStrategy,
        mode: CollisionMode,
        primitive: P,
        ty: Y,
    ) -> Self {
        Self::new_complex(strategy, mode, vec![(primitive, T::one())], ty)
    }

    /// Convenience function to create a simple collision shape with only a single given primitive,
    /// with a given local-to-model transform.
    ///
    /// # Parameters
    ///
    /// - `strategy`: The collision strategy to use for this shape.
    /// - `primitive`: The collision primitive.
    /// - `transform`: Local-to-model transform of the primitive.
    pub fn new_simple_offset(
        strategy: CollisionStrategy,
        mode: CollisionMode,
        primitive: P,
        transform: T,
    ) -> Self {
        Self::new_complex(
            strategy,
            mode,
            vec![(primitive, transform)],
            Default::default(),
        )
    }

    /// Update the cached transformed bounding box in world space coordinates.
    ///
    /// If the end transform is given, that will always be used. If the collision mode of the shape
    /// is `Continuous`, both the start and end transforms will be added to the transformed bounding
    /// box. This will make broad phase detect collisions for the whole transformation path.
    ///
    /// ## Parameters
    ///
    /// - `start`: Current model-to-world transform of the shape at the start of the frame.
    /// - `end`: Optional model-to-world transform of the shaped at the end of the frame.
    pub fn update(&mut self, start: &T, end: Option<&T>) {
        self.transformed_bound = match end {
            None => self.base_bound.transform_volume(start),
            Some(end_t) => {
                let base = self.base_bound.transform_volume(end_t);
                if self.mode == CollisionMode::Continuous {
                    base.union(&self.base_bound.transform_volume(start))
                } else {
                    base
                }
            }
        };
    }

    /// Return the current transformed bound for the shape
    ///
    pub fn bound(&self) -> &B {
        &self.transformed_bound
    }

    /// Borrow the primitives of the shape
    pub fn primitives(&self) -> &Vec<(P, T)> {
        &self.primitives
    }
}

impl<P, T, B, Y> HasBound for CollisionShape<P, T, B, Y>
where
    P: Primitive + ComputeBound<B>,
    B: Bound<Point = P::Point> + Union<B, Output = B> + Clone,
    T: Transform<P::Point>,
    Y: Default,
{
    type Bound = B;

    fn bound(&self) -> &Self::Bound {
        &self.transformed_bound
    }
}

fn get_bound<P, T, B>(primitives: &Vec<(P, T)>) -> B
where
    P: Primitive + ComputeBound<B>,
    B: Bound<Point = P::Point> + Union<B, Output = B>,
    T: Transform<P::Point>,
{
    primitives
        .iter()
        .map(|&(ref p, ref t)| p.compute_bound().transform_volume(t))
        .fold(B::empty(), |bound, b| bound.union(&b))
}
