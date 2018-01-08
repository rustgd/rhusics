use cgmath::{BaseFloat, EuclideanSpace, Rotation, VectorSpace, Zero};
use collision::Bound;
use specs::{Entity, EntityBuilder, LazyUpdate};

use core::{BodyPose, CollisionShape, ForceAccumulator, Mass, NextFrame, Primitive, RigidBody,
           Velocity};

/// Time step resource
///
/// ### Type parameters:
///
/// - `S`: Scalar
#[derive(Debug)]
#[cfg_attr(feature = "eders", derive(Serialize, Deserialize))]
pub struct DeltaTime<S>
where
    S: BaseFloat,
{
    /// Delta time since last frame
    pub delta_seconds: S,
}

/// Adds rigid body builder functions to `EntityBuilder`
pub trait WithRigidBody {
    /// Add dynamic rigid body components to entity
    ///
    /// ### Type parameters:
    ///
    /// - `P`: Collision Primitive
    /// - `Y`: Collider
    /// - `R`: Rotational quantity
    /// - `V`: Vector
    /// - `A`: Angular velocity
    /// - `I`: Inertia
    /// - `B`: Bounding volume
    fn with_dynamic_rigid_body<P, Y, R, V, A, I, B>(
        self,
        shape: CollisionShape<P, BodyPose<P::Point, R>, B, Y>,
        pose: BodyPose<P::Point, R>,
        velocity: Velocity<V, A>,
        body: RigidBody<V::Scalar>,
        mass: Mass<V::Scalar, I>,
    ) -> Self
    where
        P: Primitive + Send + Sync + 'static,
        B: Bound<Point = P::Point> + Send + Sync + 'static,
        P::Point: EuclideanSpace<Scalar = V::Scalar> + Send + Sync + 'static,
        V::Scalar: BaseFloat + Send + Sync + 'static,
        R: Rotation<P::Point> + Send + Sync + 'static,
        V: VectorSpace + Zero + Clone + Send + Sync + 'static,
        A: Copy + Zero + Clone + Send + Sync + 'static,
        Y: Send + Sync + 'static,
        I: Send + Sync + 'static;

    /// Add static rigid body components to entity
    ///
    /// ### Type parameters:
    ///
    /// - `S`: Scalar (f32 or f64)
    /// - `P`: Collision Primitive
    /// - `Y`: Collider
    /// - `R`: Rotational quantity
    /// - `I`: Inertia
    /// - `B`: Bounding volume
    fn with_static_rigid_body<S, P, Y, R, I, B>(
        self,
        shape: CollisionShape<P, BodyPose<P::Point, R>, B, Y>,
        pose: BodyPose<P::Point, R>,
        body: RigidBody<S>,
        mass: Mass<S, I>,
    ) -> Self
    where
        S: BaseFloat + Send + Sync + 'static,
        P: Primitive + Send + Sync + 'static,
        B: Bound<Point = P::Point> + Send + Sync + 'static,
        P::Point: EuclideanSpace<Scalar = S> + Send + Sync + 'static,
        R: Rotation<P::Point> + Send + Sync + 'static,
        Y: Send + Sync + 'static,
        I: Send + Sync + 'static;
}

/// Adds rigid body builder functions to `LazyUpdate`
pub trait WithLazyRigidBody {
    /// Add dynamic rigid body components to entity
    ///
    /// ### Type parameters:
    ///
    /// - `P`: Collision Primitive
    /// - `Y`: Collider
    /// - `R`: Rotational quantity
    /// - `V`: Vector
    /// - `A`: Angular velocity
    /// - `I`: Inertia
    /// - `B`: Bounding volume
    fn with_dynamic_rigid_body<P, Y, R, V, A, I, B>(
        &self,
        entity: Entity,
        shape: CollisionShape<P, BodyPose<P::Point, R>, B, Y>,
        pose: BodyPose<P::Point, R>,
        velocity: Velocity<V, A>,
        body: RigidBody<V::Scalar>,
        mass: Mass<V::Scalar, I>,
    ) where
        P: Primitive + Send + Sync + 'static,
        B: Bound<Point = P::Point> + Send + Sync + 'static,
        P::Point: EuclideanSpace<Scalar = V::Scalar> + Send + Sync + 'static,
        R: Rotation<P::Point> + Send + Sync + 'static,
        V: VectorSpace + Zero + Clone + Send + Sync + 'static,
        V::Scalar: BaseFloat + Send + Sync + 'static,
        A: Copy + Zero + Clone + Send + Sync + 'static,
        Y: Send + Sync + 'static,
        I: Send + Sync + 'static;

    /// Add static rigid body components to entity
    ///
    /// ### Type parameters:
    ///
    /// - `S`: Scalar (f32 or f64)
    /// - `P`: Collision Primitive
    /// - `Y`: Collider
    /// - `R`: Rotational quantity
    /// - `I`: Inertia
    /// - `B`: Bounding volume
    fn with_static_rigid_body<S, P, Y, R, I, B>(
        &self,
        entity: Entity,
        shape: CollisionShape<P, BodyPose<P::Point, R>, B, Y>,
        pose: BodyPose<P::Point, R>,
        body: RigidBody<S>,
        mass: Mass<S, I>,
    ) where
        P: Primitive + Send + Sync + 'static,
        B: Bound<Point = P::Point> + Send + Sync + 'static,
        P::Point: EuclideanSpace<Scalar = S> + Send + Sync + 'static,
        S: BaseFloat + Send + Sync + 'static,
        R: Rotation<P::Point> + Send + Sync + 'static,
        Y: Send + Sync + 'static,
        I: Send + Sync + 'static;
}

impl WithLazyRigidBody for LazyUpdate {
    fn with_dynamic_rigid_body<P, Y, R, V, A, I, B>(
        &self,
        entity: Entity,
        shape: CollisionShape<P, BodyPose<P::Point, R>, B, Y>,
        pose: BodyPose<P::Point, R>,
        velocity: Velocity<V, A>,
        body: RigidBody<V::Scalar>,
        mass: Mass<V::Scalar, I>,
    ) where
        P: Primitive + Send + Sync + 'static,
        B: Bound<Point = P::Point> + Send + Sync + 'static,
        P::Point: EuclideanSpace<Scalar = V::Scalar> + Send + Sync + 'static,
        R: Rotation<P::Point> + Send + Sync + 'static,
        V: VectorSpace + Zero + Clone + Send + Sync + 'static,
        V::Scalar: BaseFloat + Send + Sync + 'static,
        A: Copy + Zero + Clone + Send + Sync + 'static,
        Y: Send + Sync + 'static,
        I: Send + Sync + 'static,
    {
        self.with_static_rigid_body(entity, shape, pose, body, mass);
        self.insert(entity, velocity.clone());
        self.insert(
            entity,
            NextFrame {
                value: velocity.clone(),
            },
        );
        self.insert(entity, ForceAccumulator::<V, A>::new());
    }

    fn with_static_rigid_body<S, P, Y, R, I, B>(
        &self,
        entity: Entity,
        shape: CollisionShape<P, BodyPose<P::Point, R>, B, Y>,
        pose: BodyPose<P::Point, R>,
        body: RigidBody<S>,
        mass: Mass<S, I>,
    ) where
        P: Primitive + Send + Sync + 'static,
        B: Bound<Point = P::Point> + Send + Sync + 'static,
        P::Point: EuclideanSpace<Scalar = S> + Send + Sync + 'static,
        S: BaseFloat + Send + Sync + 'static,
        R: Rotation<P::Point> + Send + Sync + 'static,
        Y: Send + Sync + 'static,
        I: Send + Sync + 'static,
    {
        self.insert(entity, shape);
        self.insert(entity, body);
        self.insert(entity, mass);
        self.insert(entity, pose.clone());
        self.insert(entity, NextFrame { value: pose });
    }
}

impl<'a> WithRigidBody for EntityBuilder<'a> {
    fn with_dynamic_rigid_body<P, Y, R, V, A, I, B>(
        self,
        shape: CollisionShape<P, BodyPose<P::Point, R>, B, Y>,
        pose: BodyPose<P::Point, R>,
        velocity: Velocity<V, A>,
        body: RigidBody<V::Scalar>,
        mass: Mass<V::Scalar, I>,
    ) -> Self
    where
        P: Primitive + Send + Sync + 'static,
        B: Bound<Point = P::Point> + Send + Sync + 'static,
        P::Point: EuclideanSpace<Scalar = V::Scalar> + Send + Sync + 'static,
        R: Rotation<P::Point> + Send + Sync + 'static,
        V: VectorSpace + Zero + Clone + Send + Sync + 'static,
        V::Scalar: BaseFloat + Send + Sync + 'static,
        A: Copy + Clone + Zero + Send + Sync + 'static,
        Y: Send + Sync + 'static,
        I: Send + Sync + 'static,
    {
        self.with_static_rigid_body(shape, pose, body, mass)
            .with(velocity.clone())
            .with(NextFrame { value: velocity })
            .with(ForceAccumulator::<V, A>::new())
    }

    fn with_static_rigid_body<S, P, Y, R, I, B>(
        self,
        shape: CollisionShape<P, BodyPose<P::Point, R>, B, Y>,
        pose: BodyPose<P::Point, R>,
        body: RigidBody<S>,
        mass: Mass<S, I>,
    ) -> Self
    where
        S: BaseFloat + Send + Sync + 'static,
        P: Primitive + Send + Sync + 'static,
        B: Bound<Point = P::Point> + Send + Sync + 'static,
        P::Point: EuclideanSpace<Scalar = S> + Send + Sync + 'static,
        R: Rotation<P::Point> + Send + Sync + 'static,
        Y: Send + Sync + 'static,
        I: Send + Sync + 'static,
    {
        self.with(shape)
            .with(body)
            .with(mass)
            .with(pose.clone())
            .with(NextFrame { value: pose })
    }
}
