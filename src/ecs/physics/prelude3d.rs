//! 3D physics ECS

pub use ecs::collide::prelude3d::*;
pub use ecs::physics::{DeltaTime, WithLazyRigidBody, WithRigidBody};
pub use physics::prelude3d::*;

use cgmath::{Matrix3, Point3, Quaternion, Vector3};
use specs::World;

use {NextFrame, Real};
use ecs::physics::{ContactResolutionSystem, ImpulseSolverSystem, NextFrameSetupSystem};

/// Current frame integrator system for 2D
pub type ImpulseSolverSystem3 = ImpulseSolverSystem<Point3<Real>, Quaternion<Real>, Vector3<Real>>;

/// Resolution system for 2D
pub type ContactResolutionSystem3 = ContactResolutionSystem<
    Point3<Real>,
    Quaternion<Real>,
    Matrix3<Real>,
    Vector3<Real>,
    Vector3<Real>,
>;

/// Next frame setup system for 2D
pub type NextFrameSetupSystem3 = NextFrameSetupSystem<
    Point3<Real>,
    Quaternion<Real>,
    Matrix3<Real>,
    Vector3<Real>,
>;

/// Utility method for registering 3D physics and collision components and resources with
/// [`specs::World`](https://docs.rs/specs/0.9.5/specs/struct.World.html).
///
/// # Parameters
///
/// - `world`: The [world](https://docs.rs/specs/0.9.5/specs/struct.World.html)
/// to register components/resources in.
///
/// # Type parameters
///
/// - `Y`: Collision shape type, see `Collider`
pub fn register_physics<Y>(world: &mut World)
where
    Y: Send + Sync + 'static,
{
    world.add_resource(DeltaTime { delta_seconds: 0. });
    world.register::<Mass3>();
    world.register::<Velocity3>();
    world.register::<NextFrame<Velocity3>>();
    world.register::<RigidBody>();
    world.register::<ForceAccumulator3>();
    register_collision::<BodyPose3, Y>(world);
}
