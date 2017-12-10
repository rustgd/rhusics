//! 3D physics ECS

pub use ecs::collide::prelude3d::*;
pub use ecs::physics::DeltaTime;
pub use physics::prelude3d::*;

use cgmath::{Point3, Quaternion};
use shrev::EventChannel;
use specs::World;

use {NextFrame, Real};
use ecs::physics::LinearContactSolverSystem;

/// Linear contact resolve system for 3D
pub type LinearContactSolverSystem3 = LinearContactSolverSystem<Point3<Real>, Quaternion<Real>>;

/// Register required components and resources in world
pub fn world_physics_register<Y>(world: &mut World)
where
    Y: Send + Sync + 'static,
{
    world.add_resource(DeltaTime { delta_seconds: 0. });
    world.register::<Mass>();
    world.register::<Velocity3>();
    world.register::<NextFrame<Velocity3>>();
    world.add_resource(EventChannel::<ContactEvent3>::new());
    world_register::<BodyPose3, Y>(world);
}

/// Register required components and resources in world
pub fn world_physics_register_with_spatial<Y>(world: &mut World)
where
    Y: Send + Sync + 'static,
{
    world.add_resource(DeltaTime { delta_seconds: 0. });
    world.register::<Mass>();
    world.register::<Velocity3>();
    world.register::<NextFrame<Velocity3>>();
    world.add_resource(EventChannel::<ContactEvent3>::new());
    world_register_with_spatial::<BodyPose3, Y>(world);
}
