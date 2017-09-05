extern crate rhusics;
extern crate cgmath;
extern crate specs;

use cgmath::{Transform, Rotation2, Rad, Point2};
use specs::{World, RunNow};

use rhusics::collide2d::{CollisionShape2D, CollisionSystem2D, BodyPose2D, BroadBruteForce2D, GJK2D,
                world_register, Rectangle, Contacts2D, CollisionStrategy};

pub fn main() {
    let mut world = World::new();
    world_register::<BodyPose2D>(&mut world);

    world
        .create_entity()
        .with(CollisionShape2D::<BodyPose2D>::new_simple(
            CollisionStrategy::FullResolution,
            Rectangle::new(10., 10.).into(),
        ))
        .with(BodyPose2D::one());

    world
        .create_entity()
        .with(CollisionShape2D::<BodyPose2D>::new_simple(
            CollisionStrategy::FullResolution,
            Rectangle::new(10., 10.).into(),
        ))
        .with(BodyPose2D::new(
            Point2::new(3., 2.),
            Rotation2::from_angle(Rad(0.)),
        ));

    let mut system = CollisionSystem2D::<BodyPose2D>::new()
        .with_broad_phase(BroadBruteForce2D::default())
        .with_narrow_phase(GJK2D::new());
    system.run_now(&world.res);
    println!("Contacts: {:?}", *world.read_resource::<Contacts2D>());
}
