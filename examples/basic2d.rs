extern crate cgmath;
extern crate rhusics;
extern crate specs;

use cgmath::{Point2, Rad, Rotation2, Transform};
use specs::{RunNow, World};

use rhusics::ecs::collide::prelude2d::{world_register, BasicCollisionSystem2, BodyPose2,
                                       BroadBruteForce2, CollisionMode, CollisionShape2,
                                       CollisionStrategy, Contacts2, GJK2, Rectangle};

pub fn main() {
    let mut world = World::new();
    world_register::<BodyPose2>(&mut world);

    world
        .create_entity()
        .with(CollisionShape2::<BodyPose2>::new_simple(
            CollisionStrategy::FullResolution,
            CollisionMode::Discrete,
            Rectangle::new(10., 10.).into(),
        ))
        .with(BodyPose2::one());

    world
        .create_entity()
        .with(CollisionShape2::<BodyPose2>::new_simple(
            CollisionStrategy::FullResolution,
            CollisionMode::Discrete,
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
