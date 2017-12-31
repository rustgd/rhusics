//! 3D physics ECS

pub use ecs::collide::prelude3d::*;
pub use ecs::physics::{DeltaTime, WithLazyRigidBody, WithRigidBody};
pub use physics::prelude3d::*;

use cgmath::{Matrix3, Point3, Quaternion, Vector3};
use collision::Aabb3;
use collision::dbvt::TreeValueWrapped;
use collision::primitive::Primitive3;
use specs::{Entity, World};

use Real;
use ecs::physics::{ContactResolutionSystem, ImpulseSolverSystem, NextFrameSetupSystem};
use ecs::WithRhusics;

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
    Y: Collider + Default + Send + Sync + 'static,
{
    world.register_physics::<
        Primitive3<Real>,
        Aabb3<Real>, Quaternion<Real>,
        TreeValueWrapped<Entity, Aabb3<Real>>,
        Y,
        Vector3<Real>,
        Vector3<Real>,
        Matrix3<Real>
    >();
}
