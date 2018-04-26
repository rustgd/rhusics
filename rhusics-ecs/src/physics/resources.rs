use std::marker;

use cgmath::{BaseFloat, EuclideanSpace, Rotation, VectorSpace, Zero};
use collision::Bound;
use specs::prelude::{Component, Entity, EntityBuilder, SystemData, World, WriteStorage};

use core::{CollisionShape, ForceAccumulator, Mass, NextFrame, PhysicsTime, Pose, Primitive,
           RigidBody, Velocity};

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

impl<S> Default for DeltaTime<S>
where
    S: BaseFloat,
{
    fn default() -> Self {
        DeltaTime {
            delta_seconds: S::zero(),
        }
    }
}

impl<S> PhysicsTime<S> for DeltaTime<S> where S: BaseFloat {
    fn delta_seconds(&self) -> S {
        self.delta_seconds
    }
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
    /// - `T`: Transform
    fn with_dynamic_rigid_body<P, Y, R, V, A, I, B, T>(
        self,
        shape: CollisionShape<P, T, B, Y>,
        pose: T,
        velocity: Velocity<V, A>,
        body: RigidBody<V::Scalar>,
        mass: Mass<V::Scalar, I>,
    ) -> Self
    where
        T: Pose<P::Point, R> + Clone + Component + Send + Sync + 'static,
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
    /// - `T`: Transform
    fn with_static_rigid_body<S, P, Y, R, I, B, T>(
        self,
        shape: CollisionShape<P, T, B, Y>,
        pose: T,
        body: RigidBody<S>,
        mass: Mass<S, I>,
    ) -> Self
    where
        T: Pose<P::Point, R> + Clone + Component + Send + Sync + 'static,
        S: BaseFloat + Send + Sync + 'static,
        P: Primitive + Send + Sync + 'static,
        B: Bound<Point = P::Point> + Send + Sync + 'static,
        P::Point: EuclideanSpace<Scalar = S> + Send + Sync + 'static,
        R: Rotation<P::Point> + Send + Sync + 'static,
        Y: Send + Sync + 'static,
        I: Send + Sync + 'static;
}

impl<'a> WithRigidBody for EntityBuilder<'a> {
    fn with_dynamic_rigid_body<P, Y, R, V, A, I, B, T>(
        self,
        shape: CollisionShape<P, T, B, Y>,
        pose: T,
        velocity: Velocity<V, A>,
        body: RigidBody<V::Scalar>,
        mass: Mass<V::Scalar, I>,
    ) -> Self
    where
        T: Pose<P::Point, R> + Clone + Component + Send + Sync + 'static,
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
        self.with_static_rigid_body(shape, pose.clone(), body, mass)
            .with(NextFrame { value: pose })
            .with(velocity.clone())
            .with(NextFrame { value: velocity })
            .with(ForceAccumulator::<V, A>::new())
    }

    fn with_static_rigid_body<S, P, Y, R, I, B, T>(
        self,
        shape: CollisionShape<P, T, B, Y>,
        pose: T,
        body: RigidBody<S>,
        mass: Mass<S, I>,
    ) -> Self
    where
        T: Pose<P::Point, R> + Clone + Component + Send + Sync + 'static,
        S: BaseFloat + Send + Sync + 'static,
        P: Primitive + Send + Sync + 'static,
        B: Bound<Point = P::Point> + Send + Sync + 'static,
        P::Point: EuclideanSpace<Scalar = S> + Send + Sync + 'static,
        R: Rotation<P::Point> + Send + Sync + 'static,
        Y: Send + Sync + 'static,
        I: Send + Sync + 'static,
    {
        self.with(shape).with(body).with(mass).with(pose)
    }
}

/// SystemData for easier creation of rigid bodies.
///
/// ### Type parameters:
///
/// - `P`: Collision Primitive
/// - `Y`: Collider
/// - `R`: Rotational quantity
/// - `V`: Linear velocity
/// - `A`: Angular velocity
/// - `I`: Inertia
/// - `B`: Bounding volume
/// - `T`: Transform
#[derive(SystemData)]
pub struct RigidBodyParts<'a, P, Y, R, V, A, I, B, T>
where
    T: Pose<P::Point, R> + Clone + Component + Send + Sync + 'static,
    P: Primitive + Send + Sync + 'static,
    B: Bound<Point = P::Point> + Send + Sync + 'static,
    P::Point: EuclideanSpace<Scalar = V::Scalar> + Send + Sync + 'static,
    V::Scalar: BaseFloat + Send + Sync + 'static,
    V: VectorSpace + Zero + Clone + Send + Sync + 'static,
    A: Copy + Zero + Clone + Send + Sync + 'static,
    Y: Send + Sync + 'static,
    I: Send + Sync + 'static,
    R: Rotation<P::Point> + Send + Sync + 'static,
{
    /// Collision shapes
    pub shapes: WriteStorage<'a, CollisionShape<P, T, B, Y>>,
    /// Body transforms
    pub poses: WriteStorage<'a, T>,
    /// Bodies
    pub bodies: WriteStorage<'a, RigidBody<V::Scalar>>,
    /// Mass
    pub masses: WriteStorage<'a, Mass<V::Scalar, I>>,
    /// Velocity
    pub velocities: WriteStorage<'a, Velocity<V, A>>,
    /// Next frame transform
    pub next_poses: WriteStorage<'a, NextFrame<T>>,
    /// Next frame velocity
    pub next_velocities: WriteStorage<'a, NextFrame<Velocity<V, A>>>,
    /// Forces
    pub forces: WriteStorage<'a, ForceAccumulator<V, A>>,
    m: marker::PhantomData<R>,
}

impl<'a, P, Y, R, V, A, I, B, T> RigidBodyParts<'a, P, Y, R, V, A, I, B, T>
where
    T: Pose<P::Point, R> + Clone + Component + Send + Sync + 'static,
    P: Primitive + Send + Sync + 'static,
    B: Bound<Point = P::Point> + Send + Sync + 'static,
    P::Point: EuclideanSpace<Scalar = V::Scalar> + Send + Sync + 'static,
    V::Scalar: BaseFloat + Send + Sync + 'static,
    V: VectorSpace + Zero + Clone + Send + Sync + 'static,
    A: Copy + Zero + Clone + Send + Sync + 'static,
    Y: Send + Sync + 'static,
    I: Send + Sync + 'static,
    R: Rotation<P::Point> + Send + Sync + 'static,
{
    /// Extract rigid body storage from `World`
    pub fn new(world: &'a World) -> Self {
        Self::fetch(&world.res)
    }

    /// Setup static rigid body for given entity.
    pub fn static_body(
        &mut self,
        entity: Entity,
        shape: CollisionShape<P, T, B, Y>,
        pose: T,
        body: RigidBody<V::Scalar>,
        mass: Mass<V::Scalar, I>,
    ) {
        self.shapes.insert(entity, shape);
        self.poses.insert(entity, pose);
        self.bodies.insert(entity, body);
        self.masses.insert(entity, mass);
    }

    /// Setup dynamic rigid body for given entity.
    pub fn dynamic_body(
        &mut self,
        entity: Entity,
        shape: CollisionShape<P, T, B, Y>,
        pose: T,
        velocity: Velocity<V, A>,
        body: RigidBody<V::Scalar>,
        mass: Mass<V::Scalar, I>,
    ) {
        self.static_body(entity, shape, pose.clone(), body, mass);
        self.next_poses.insert(entity, NextFrame { value: pose });
        self.velocities.insert(entity, velocity.clone());
        self.next_velocities
            .insert(entity, NextFrame { value: velocity });
        self.forces.insert(entity, ForceAccumulator::<V, A>::new());
    }
}

#[cfg(test)]
mod tests {
    use super::RigidBodyParts;
    use cgmath::{Matrix3, Quaternion, Vector3};
    use collision::Aabb3;
    use collision::primitive::Primitive3;
    use core::collide3d::BodyPose3;
    use specs::prelude::{SystemData, World};

    type RigidBodyPartsTest<'a> = RigidBodyParts<
        'a,
        Primitive3<f32>,
        (),
        Quaternion<f32>,
        Vector3<f32>,
        Vector3<f32>,
        Matrix3<f32>,
        Aabb3<f32>,
        BodyPose3<f32>,
    >;

    #[test]
    fn test() {
        let mut world = World::new();
        RigidBodyPartsTest::setup(&mut world.res);
        RigidBodyPartsTest::new(&world);
    }
}
