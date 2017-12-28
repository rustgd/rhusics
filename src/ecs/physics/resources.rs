use cgmath::{Rotation, VectorSpace, Zero};
use collision::Aabb;
use specs::{Component, DenseVecStorage, Entity, EntityBuilder, LazyUpdate};

use {BodyPose, NextFrame, Real};
use collide::{CollisionShape, Primitive};
use physics::{ForceAccumulator, Mass, RigidBody, Velocity};

impl<V, A> Component for Velocity<V, A>
where
    V: Send + Sync + 'static + Clone,
    A: Send + Sync + 'static + Clone,
{
    type Storage = DenseVecStorage<Self>;
}

impl<I> Component for Mass<I>
where
    I: Send + Sync + 'static,
{
    type Storage = DenseVecStorage<Self>;
}

impl Component for RigidBody {
    type Storage = DenseVecStorage<Self>;
}

impl<F, A> Component for ForceAccumulator<F, A>
where
    F: Send + Sync + 'static,
    A: Send + Sync + 'static,
{
    type Storage = DenseVecStorage<Self>;
}

/// Time step
pub struct DeltaTime {
    /// Delta time since last frame
    pub delta_seconds: Real,
}

/// Adds rigid body builder functions to `EntityBuilder`
pub trait WithRigidBody {
    /// Add dynamic rigid body components to entity
    fn with_dynamic_rigid_body<P, Y, R, V, A, I>(
        self,
        shape: CollisionShape<P, BodyPose<P::Point, R>, Y>,
        pose: BodyPose<P::Point, R>,
        velocity: Velocity<V, A>,
        body: RigidBody,
        mass: Mass<I>,
    ) -> Self
    where
        P: Primitive + Send + Sync + 'static,
        P::Aabb: Aabb<Scalar = Real> + Send + Sync + 'static,
        P::Point: Send + Sync + 'static,
        R: Rotation<P::Point> + Send + Sync + 'static,
        V: VectorSpace<Scalar = Real> + Zero + Clone + Send + Sync + 'static,
        A: Copy + Zero + Clone + Send + Sync + 'static,
        Y: Send + Sync + 'static,
        I: Send + Sync + 'static;

    /// Add static rigid body components to entity
    fn with_static_rigid_body<P, Y, R, I>(
        self,
        shape: CollisionShape<P, BodyPose<P::Point, R>, Y>,
        pose: BodyPose<P::Point, R>,
        body: RigidBody,
        mass: Mass<I>,
    ) -> Self
    where
        P: Primitive + Send + Sync + 'static,
        P::Aabb: Aabb<Scalar = Real> + Send + Sync + 'static,
        P::Point: Send + Sync + 'static,
        R: Rotation<P::Point> + Send + Sync + 'static,
        Y: Send + Sync + 'static,
        I: Send + Sync + 'static;
}

/// Adds rigid body builder functions to `LazyUpdate`
pub trait WithLazyRigidBody {
    /// Add dynamic rigid body components to entity
    fn with_dynamic_rigid_body<P, Y, R, V, A, I>(
        &self,
        entity: Entity,
        shape: CollisionShape<P, BodyPose<P::Point, R>, Y>,
        pose: BodyPose<P::Point, R>,
        velocity: Velocity<V, A>,
        body: RigidBody,
        mass: Mass<I>,
    ) where
        P: Primitive + Send + Sync + 'static,
        P::Aabb: Aabb<Scalar = Real> + Send + Sync + 'static,
        P::Point: Send + Sync + 'static,
        R: Rotation<P::Point> + Send + Sync + 'static,
        V: VectorSpace<Scalar = Real> + Zero + Clone + Send + Sync + 'static,
        A: Copy + Zero + Clone + Send + Sync + 'static,
        Y: Send + Sync + 'static,
        I: Send + Sync + 'static;

    /// Add static rigid body components to entity
    fn with_static_rigid_body<P, Y, R, I>(
        &self,
        entity: Entity,
        shape: CollisionShape<P, BodyPose<P::Point, R>, Y>,
        pose: BodyPose<P::Point, R>,
        body: RigidBody,
        mass: Mass<I>,
    ) where
        P: Primitive + Send + Sync + 'static,
        P::Aabb: Aabb<Scalar = Real> + Send + Sync + 'static,
        P::Point: Send + Sync + 'static,
        R: Rotation<P::Point> + Send + Sync + 'static,
        Y: Send + Sync + 'static,
        I: Send + Sync + 'static;
}

impl WithLazyRigidBody for LazyUpdate {
    fn with_dynamic_rigid_body<P, Y, R, V, A, I>(
        &self,
        entity: Entity,
        shape: CollisionShape<P, BodyPose<P::Point, R>, Y>,
        pose: BodyPose<P::Point, R>,
        velocity: Velocity<V, A>,
        body: RigidBody,
        mass: Mass<I>,
    ) where
        P: Primitive + Send + Sync + 'static,
        P::Aabb: Aabb<Scalar = Real> + Send + Sync + 'static,
        P::Point: Send + Sync + 'static,
        R: Rotation<P::Point> + Send + Sync + 'static,
        V: VectorSpace<Scalar = Real> + Zero + Clone + Send + Sync + 'static,
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

    fn with_static_rigid_body<P, Y, R, I>(
        &self,
        entity: Entity,
        shape: CollisionShape<P, BodyPose<P::Point, R>, Y>,
        pose: BodyPose<P::Point, R>,
        body: RigidBody,
        mass: Mass<I>,
    ) where
        P: Primitive + Send + Sync + 'static,
        P::Aabb: Aabb<Scalar = Real> + Send + Sync + 'static,
        P::Point: Send + Sync + 'static,
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
    fn with_dynamic_rigid_body<P, Y, R, V, A, I>(
        self,
        shape: CollisionShape<P, BodyPose<P::Point, R>, Y>,
        pose: BodyPose<P::Point, R>,
        velocity: Velocity<V, A>,
        body: RigidBody,
        mass: Mass<I>,
    ) -> Self
    where
        P: Primitive + Send + Sync + 'static,
        P::Aabb: Aabb<Scalar = Real> + Send + Sync + 'static,
        P::Point: Send + Sync + 'static,
        R: Rotation<P::Point> + Send + Sync + 'static,
        V: VectorSpace<Scalar = Real> + Zero + Clone + Send + Sync + 'static,
        A: Copy + Clone + Zero + Send + Sync + 'static,
        Y: Send + Sync + 'static,
        I: Send + Sync + 'static,
    {
        self.with_static_rigid_body(shape, pose, body, mass)
            .with(velocity.clone())
            .with(NextFrame { value: velocity })
            .with(ForceAccumulator::<V, A>::new())
    }

    fn with_static_rigid_body<P, Y, R, I>(
        self,
        shape: CollisionShape<P, BodyPose<P::Point, R>, Y>,
        pose: BodyPose<P::Point, R>,
        body: RigidBody,
        mass: Mass<I>,
    ) -> Self
    where
        P: Primitive + Send + Sync + 'static,
        P::Aabb: Aabb<Scalar = Real> + Send + Sync + 'static,
        P::Point: Send + Sync + 'static,
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
