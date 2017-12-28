extern crate cgmath;
extern crate rhusics;
extern crate specs;
extern crate shrev;

use cgmath::{Point2, Rad, Rotation2, Transform};
use specs::{RunNow, World};
use shrev::EventChannel;

use rhusics::ecs::collide::prelude2d::{world_register, BasicCollisionSystem2, BodyPose2,
                                       BroadBruteForce2, CollisionMode, CollisionShape2,
                                       CollisionStrategy, GJK2, Rectangle,
                                       ContactEvent2};

pub fn main() {
    let mut world = World::new();
    world_register::<BodyPose2, ()>(&mut world);

    let mut reader_1 = world
        .write_resource::<EventChannel<ContactEvent2>>()
        .register_reader();

    world
        .create_entity()
        .with(CollisionShape2::<BodyPose2, ()>::new_simple(
            CollisionStrategy::FullResolution,
            CollisionMode::Discrete,
            Rectangle::new(10., 10.).into(),
        ))
        .with(BodyPose2::one());

    world
        .create_entity()
        .with(CollisionShape2::<BodyPose2, ()>::new_simple(
            CollisionStrategy::FullResolution,
            CollisionMode::Discrete,
            Rectangle::new(10., 10.).into(),
        ))
        .with(BodyPose2::new(
            Point2::new(3., 2.),
            Rotation2::from_angle(Rad(0.)),
        ));

    let mut system = BasicCollisionSystem2::<BodyPose2, ()>::new()
        .with_broad_phase(BroadBruteForce2::default())
        .with_narrow_phase(GJK2::new());
    system.run_now(&world.res);
    println!(
        "Contacts: {:?}",
        world
            .read_resource::<EventChannel<ContactEvent2>>()
            .read(&mut reader_1)
            .collect::<Vec<_>>()
    );
}
