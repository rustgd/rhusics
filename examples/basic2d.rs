extern crate rhusics;
extern crate cgmath;
extern crate specs;

use cgmath::{Transform, Rotation2, Rad, Point2};
use specs::{World, RunNow};

use rhusics::collide2d::{CollisionShape2, BasicCollisionSystem2, BodyPose2, BroadBruteForce2,
                         GJK2, world_register, Rectangle, Contacts2, CollisionStrategy};

pub fn main() {
    let mut world = World::new();
    world_register::<BodyPose2>(&mut world);

    world
        .create_entity()
        .with(CollisionShape2::<BodyPose2>::new_simple(
            CollisionStrategy::FullResolution,
            Rectangle::new(10., 10.).into(),
        ))
        .with(BodyPose2::one());

    world
        .create_entity()
        .with(CollisionShape2::<BodyPose2>::new_simple(
            CollisionStrategy::FullResolution,
            Rectangle::new(10., 10.).into(),
        ))
        .with(BodyPose2::new(
            Point2::new(3., 2.),
            Rotation2::from_angle(Rad(0.)),
        ));

    let mut system = BasicCollisionSystem2::<BodyPose2>::new()
        .with_broad_phase(BroadBruteForce2::default())
        .with_narrow_phase(GJK2::new());
    system.run_now(&world.res);
    println!("Contacts: {:?}", *world.read_resource::<Contacts2>());
}
