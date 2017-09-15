//! Generic data structures and algorithms for collision detection

pub mod broad;
pub mod narrow;
pub mod primitive2d;
pub mod primitive3d;
pub mod ecs;

use std::fmt::Debug;

use cgmath::prelude::*;
use collision::{Aabb, MinMax, Union};
use collision::dbvt::TreeValue;

use self::broad::BroadCollisionData;
use {Pose, Real};

/// Collision strategy to use for collisions.
///
/// This is used both to specify what collision strategy to use for each shape, and also each
/// found contact will have this returned on it, detailing what data is relevant in the
/// [`Contact`](struct.Contact.html).
#[derive(Debug, PartialEq, Clone)]
pub enum CollisionStrategy {
    /// Compute full contact manifold for the collision
    FullResolution,

    /// Only report that a collision occurred, skip computing contact information for the collision.
    CollisionOnly,
}

/// Contains all the contacts found between two bodies in a single pass.
///
/// # Type parameters
///
/// - `ID`: The ID type of the body. This is supplied by the user of the library. In the ECS case,
///         this will be [`Entity`](https://docs.rs/specs/0.9.5/specs/struct.Entity.html).
/// - `V`: cgmath vector type
#[derive(Debug)]
pub struct ContactSet<ID, P>
where
    P: EuclideanSpace,
    P::Diff: Debug,
{
    /// The ids of the two colliding bodies
    pub bodies: (ID, ID),

    /// The list of contacts between the colliding bodies
    pub contacts: Vec<Contact<P>>,
}

impl<ID, P> ContactSet<ID, P>
where
    ID: Clone + Debug,
    P: EuclideanSpace,
    P::Diff: VectorSpace + Zero + Debug,
{
    /// Create a new contact set
    pub fn new(bodies: (ID, ID), contacts: Vec<Contact<P>>) -> Self {
        Self { bodies, contacts }
    }

    /// Convenience function to create a contact set with a single [`Contact`](struct.Contact.html).
    pub fn new_single(strategy: CollisionStrategy, bodies: (ID, ID)) -> Self {
        Self::new(bodies, vec![Contact::new(strategy)])
    }
}

/// Contact manifold for a single collision contact point.
///
/// # Type parameters
///
/// - `P`: cgmath point type
#[derive(Debug)]
pub struct Contact<P: EuclideanSpace> {
    /// The collision strategy used for this contact.
    pub strategy: CollisionStrategy,

    /// The collision normal. Only applicable if the collision strategy is not `CollisionOnly`
    pub normal: P::Diff,

    /// The penetration depth. Only applicable if the collision strategy is not `CollisionOnly`
    pub penetration_depth: P::Scalar,

    /// The contact point. Only applicable if the collision strategy is not `CollisionOnly`
    pub contact_point: P,
}

impl<P> Contact<P>
where
    P: EuclideanSpace,
    P::Diff: VectorSpace + Zero,
{
    /// Create a new contact manifold, with default collision normal and penetration depth
    pub fn new(strategy: CollisionStrategy) -> Self {
        Self::new_impl(strategy, P::Diff::zero(), P::Scalar::zero())
    }

    /// Create a new contact manifold, with the given collision normal and penetration depth
    pub fn new_impl(strategy: CollisionStrategy, normal: P::Diff, penetration_depth: P::Scalar) -> Self {
        Self {
            strategy,
            normal,
            penetration_depth,
            contact_point: P::from_value(P::Scalar::zero()),
        }
    }
}

/// Trait detailing a collision primitive. These are the building blocks for all collision shapes.
///
/// See [primitive2d](primitive2d/index.html) and [primitive3d](primitive3d/index.html)
/// for more information about supported primitives.
///
pub trait Primitive: Debug + Clone + Send + Sync {
    /// Vector type used by the primitive
    type Vector: VectorSpace<Scalar = Real> + ElementWise + Array<Element = Real>;

    /// Point type used by the primitive
    type Point: EuclideanSpace<Scalar = Real, Diff = Self::Vector> + MinMax;

    /// Bounding box type used by the primitive
    type Aabb: Aabb<Scalar = Real, Diff = Self::Vector, Point = Self::Point>
        + Clone
        + Union<Self::Aabb, Output = Self::Aabb>;

    /// Get the furthest point from the origin on the shape in a given direction.
    ///
    /// # Parameters
    ///
    /// - `direction`: The search direction in world space.
    /// - `transform`: The current local to world transform for this shape.
    ///
    /// # Returns
    ///
    /// Returns the point that is furthest away from the origin.
    ///
    /// # Type parameters
    ///
    /// - `P`: Transform type
    fn get_far_point<T>(&self, direction: &Self::Vector, transform: &T) -> Self::Point
    where
        T: Pose<Self::Point>;

    /// Get the bounding box of the primitive in local space coordinates.
    fn get_bound(&self) -> Self::Aabb;
}

/// Collision primitive with local to model transform.
///
/// Contains cached information about the bounding box of the contained
/// [`Primitive`](trait.Primitive.html), both the base bounding box in model space coordinates,
/// and the transformed bounding box in world space coordinates.
#[derive(Debug, Clone)]
pub struct CollisionPrimitive<P, T>
where
    P: Primitive,
{
    local_transform: T,
    base_bound: P::Aabb,
    transformed_bound: P::Aabb,
    primitive: P,
}

impl<P, T> CollisionPrimitive<P, T>
where
    P: Primitive,
    T: Pose<P::Point>,
{
    /// Create a new collision primitive, with an identity local-to-model transform.
    ///
    /// # Parameters
    ///
    /// - `primitive`: The primitive to use.
    pub fn new(primitive: P) -> Self {
        Self::new_impl(primitive, T::one())
    }

    /// Create a new collision primitive, with a supplied local-to-model transform.
    ///
    /// # Parameters:
    ///
    /// - `primitive`: The primitive.
    /// - `local_transform`: The local-to-model transform.
    pub fn new_impl(primitive: P, local_transform: T) -> Self {
        let bound = primitive.get_bound().transform(&local_transform);
        Self {
            local_transform,
            base_bound: bound.clone(),
            transformed_bound: bound,
            primitive,
        }
    }

    /// Update the cached bounding box in world space coordinates, by applying a new transformation
    /// on the base bounding box in model space coordinates.
    ///
    /// # Parameters
    ///
    /// - `transform`: Model-to-world transform.
    pub fn update(&mut self, transform: &T) {
        self.transformed_bound = self.base_bound.transform(transform)
    }

    /// Get the furthest point away from the origin on the primitive, in a given search direction.
    ///
    /// # Parameters
    ///
    /// - `direction`: The search direction.
    /// - `transform`: Current model-to-world transform for the shape.
    ///
    /// # Returns
    ///
    /// Returns the furthest point on the primitive in the given search direction, after applying
    /// the given model-to-world transformation.
    pub fn get_far_point(&self, direction: &P::Vector, transform: &T) -> P::Point {
        let t = transform.concat(&self.local_transform);
        self.primitive.get_far_point(direction, &t)
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
/// Also have details about what collision strategy to use for contact resolution with this shape.
#[derive(Debug, Clone)]
pub struct CollisionShape<P, T>
where
    P: Primitive,
{
    /// Enable/Disable collision detection for this shape
    pub enabled: bool,
    base_bound: P::Aabb,
    transformed_bound: P::Aabb,
    primitives: Vec<CollisionPrimitive<P, T>>,
    strategy: CollisionStrategy,
}

impl<P, T> CollisionShape<P, T>
where
    P: Primitive,
    T: Pose<P::Point>,
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
    pub fn new_complex(
        strategy: CollisionStrategy,
        primitives: Vec<CollisionPrimitive<P, T>>,
    ) -> Self {
        let bound = get_bound(&primitives);
        Self {
            base_bound: bound.clone(),
            primitives,
            enabled: true,
            transformed_bound: bound,
            strategy,
        }
    }

    /// Convenience function to create a simple collision shape with only a single given primitive,
    /// with no local-to-model transform.
    ///
    /// # Parameters
    ///
    /// - `strategy`: The collision strategy to use for this shape.
    /// - `primitive`: The collision primitive.
    pub fn new_simple(strategy: CollisionStrategy, primitive: P) -> Self {
        Self::new_complex(strategy, vec![CollisionPrimitive::new(primitive)])
    }

    /// Convenience function to create a simple collision shape with only a single given primitive,
    /// with a given local-to-model transform.
    ///
    /// # Parameters
    ///
    /// - `strategy`: The collision strategy to use for this shape.
    /// - `primitive`: The collision primitive.
    /// - `transform`: Local-to-model transform of the primitive.
    pub fn new_simple_offset(strategy: CollisionStrategy, primitive: P, transform: T) -> Self {
        Self::new_complex(
            strategy,
            vec![CollisionPrimitive::new_impl(primitive, transform)],
        )
    }

    /// Update the cached transformed bounding box in world space coordinates.
    ///
    /// # Parameters
    ///
    /// - `transform`: Current model-to-world transform of the shape.
    pub fn update(&mut self, transform: &T) {
        self.transformed_bound = self.base_bound.transform(transform);
        for mut primitive in &mut self.primitives {
            primitive.update(transform)
        }
    }

    /// Return the current transformed bound for the shape
    ///
    pub fn bound(&self) -> &P::Aabb {
        &self.transformed_bound
    }
}

/// Shape wrapper for use with containers
#[derive(Debug, Clone)]
pub struct ContainerShapeWrapper<ID, P>
where
    P: Primitive,
    P::Aabb: Debug,
    P::Vector: VectorSpace + Debug,
{
    /// The id
    pub id: ID,

    /// The bounding volume
    pub bound: P::Aabb,
    fat_factor: P::Vector,
}

impl<ID, P> ContainerShapeWrapper<ID, P>
where
    P: Primitive,
    P::Aabb: Debug + Clone,
    P::Vector: VectorSpace<Scalar = Real> + Debug,
{
    /// Create a new shape
    pub fn new_impl(id: ID, bound: &P::Aabb, fat_factor: P::Vector) -> Self {
        Self {
            id,
            bound: bound.clone(),
            fat_factor,
        }
    }

    /// Create a new shape
    pub fn new(id: ID, bound: &P::Aabb) -> Self {
        Self::new_impl(id, bound, P::Vector::from_value(Real::one()))
    }
}

impl<ID, P> TreeValue for ContainerShapeWrapper<ID, P>
where
    ID: Clone + Debug,
    P: Primitive,
    P::Aabb: Aabb + Debug,
    P::Vector: VectorSpace + Debug,
{
    type Bound = P::Aabb;

    fn bound(&self) -> &Self::Bound {
        &self.bound
    }

    fn fat_bound(&self) -> Self::Bound {
        self.bound.add_margin(self.fat_factor)
    }
}

impl<ID, P> BroadCollisionData for ContainerShapeWrapper<ID, P>
where
    P: Primitive,
    P::Aabb: Debug,
    P::Vector: VectorSpace + Debug,
{
    type Id = ID;
    type Bound = P::Aabb;

    fn id(&self) -> &ID {
        &self.id
    }

    fn bound(&self) -> &P::Aabb {
        &self.bound
    }
}

// Used for data coming out of the collision-rs DBVT.
impl<ID, P> BroadCollisionData for (usize, ContainerShapeWrapper<ID, P>)
    where
        P: Primitive,
        P::Aabb: Debug,
        P::Vector: VectorSpace + Debug,
{
    type Id = ID;
    type Bound = P::Aabb;

    fn id(&self) -> &ID {
        &self.1.id
    }

    fn bound(&self) -> &P::Aabb {
        &self.1.bound
    }
}

fn get_bound<P, T>(primitives: &Vec<CollisionPrimitive<P, T>>) -> P::Aabb
where
    P: Primitive,
{
    primitives.iter().map(|p| &p.base_bound).fold(
        P::Aabb::zero(),
        |bound, b| {
            bound.union(b)
        },
    )
}
