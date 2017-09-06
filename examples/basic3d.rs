extern crate rhusics;
extern crate cgmath;
extern crate specs;

use cgmath::{Transform, Rotation3, Rad, Point3, Quaternion};
use specs::{World, RunNow};

use rhusics::collide3d::{CollisionShape3D, CollisionSystem3D, BodyPose3D, BroadBruteForce3D,
                         GJK3D, world_register, Cuboid, Contacts3D, CollisionStrategy};

pub fn main() {
    let mut world = World::new();
    world_register::<BodyPose3D>(&mut world);

    world
        .create_entity()
        .with(CollisionShape3D::<BodyPose3D>::new_simple(
            CollisionStrategy::FullResolution,
            Cuboid::new(10., 10., 10.).into(),
        ))
        .with(BodyPose3D::one());

    world
        .create_entity()
        .with(CollisionShape3D::<BodyPose3D>::new_simple(
            CollisionStrategy::FullResolution,
            Cuboid::new(10., 10., 10.).into(),
        ))
        .with(BodyPose3D::new(
            Point3::new(3., 2., 0.),
            Quaternion::from_angle_z(Rad(0.)),
        ));

    let mut system = CollisionSystem3D::<BodyPose3D>::new()
        .with_broad_phase(BroadBruteForce3D::default())
        .with_narrow_phase(GJK3D::new());
    system.run_now(&world.res);
    println!("Contacts: {:?}", *world.read_resource::<Contacts3D>());
}
