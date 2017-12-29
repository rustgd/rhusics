extern crate cgmath;
extern crate rhusics;
extern crate shrev;
extern crate specs;

use cgmath::{Point2, Rad, Rotation2, Transform};
use shrev::EventChannel;
use specs::{RunNow, World};

use rhusics::ecs::collide::prelude2d::{register_collision, BasicCollisionSystem2, BodyPose2,
                                       BroadBruteForce2, CollisionMode, CollisionShape2,
                                       CollisionStrategy, ContactEvent2, GJK2, Rectangle};

pub fn main() {
    let mut world = World::new();
    register_collision::<BodyPose2, ()>(&mut world);

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
