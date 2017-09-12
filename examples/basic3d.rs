extern crate rhusics;
extern crate cgmath;
extern crate specs;

use cgmath::{Transform, Rotation3, Rad, Point3, Quaternion};
use specs::{World, RunNow};

use rhusics::collide3d::{CollisionShape3, BasicCollisionSystem3, BodyPose3, BroadBruteForce3,
                         GJK3, world_register, Cuboid, Contacts3, CollisionStrategy};

pub fn main() {
    let mut world = World::new();
    world_register::<BodyPose3>(&mut world);

    world
        .create_entity()
        .with(CollisionShape3::<BodyPose3>::new_simple(
            CollisionStrategy::FullResolution,
            Cuboid::new(10., 10., 10.).into(),
        ))
        .with(BodyPose3::one());

    world
        .create_entity()
        .with(CollisionShape3::<BodyPose3>::new_simple(
            CollisionStrategy::FullResolution,
            Cuboid::new(10., 10., 10.).into(),
        ))
        .with(BodyPose3::new(
            Point3::new(3., 2., 0.),
            Quaternion::from_angle_z(Rad(0.)),
        ));

    let mut system = BasicCollisionSystem3::<BodyPose3>::new()
        .with_broad_phase(BroadBruteForce3::default())
        .with_narrow_phase(GJK3::new());
    system.run_now(&world.res);
    println!("Contacts: {:?}", *world.read_resource::<Contacts3>());
}
