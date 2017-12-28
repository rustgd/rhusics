extern crate cgmath;
extern crate rhusics;
extern crate specs;
extern crate shrev;

use cgmath::{Point3, Quaternion, Rad, Rotation3, Transform};
use specs::{RunNow, World};
use shrev::EventChannel;

use rhusics::ecs::collide::prelude3d::{world_register, BasicCollisionSystem3, BodyPose3,
                                       BroadBruteForce3, CollisionMode, CollisionShape3,
                                       CollisionStrategy, ContactEvent3, Cuboid, GJK3};

pub fn main() {
    let mut world = World::new();
    world_register::<BodyPose3, ()>(&mut world);

    let mut reader_1 = world
        .write_resource::<EventChannel<ContactEvent3>>()
        .register_reader();

    world
        .create_entity()
        .with(CollisionShape3::<BodyPose3, ()>::new_simple(
            CollisionStrategy::FullResolution,
            CollisionMode::Discrete,
            Cuboid::new(10., 10., 10.).into(),
        ))
        .with(BodyPose3::one());

    world
        .create_entity()
        .with(CollisionShape3::<BodyPose3, ()>::new_simple(
            CollisionStrategy::FullResolution,
            CollisionMode::Discrete,
            Cuboid::new(10., 10., 10.).into(),
        ))
        .with(BodyPose3::new(
            Point3::new(3., 2., 0.),
            Quaternion::from_angle_z(Rad(0.)),
        ));

    let mut system = BasicCollisionSystem3::<BodyPose3, ()>::new()
        .with_broad_phase(BroadBruteForce3::default())
        .with_narrow_phase(GJK3::new());
    system.run_now(&world.res);
    println!(
        "Contacts: {:?}",
        world
            .read_resource::<EventChannel<ContactEvent3>>()
            .read(&mut reader_1)
            .collect::<Vec<_>>()
    );
}
