extern crate cgmath;
extern crate collision;
extern crate rhusics;
extern crate shrev;
extern crate specs;

use cgmath::{Point2, Rad, Rotation2, Transform, Vector2};
use collision::Ray2;
use collision::dbvt::query_ray_closest;
use shrev::EventChannel;
use specs::{Fetch, RunNow, System, World};

use rhusics::ecs::physics::prelude2d::{register_physics, BodyPose2, CollisionMode,
                                       CollisionShape2, CollisionStrategy, ContactEvent2,
                                       ContactResolutionSystem2, DynamicBoundingVolumeTree2, GJK2,
                                       CurrentFrameUpdateSystem2, Mass2, NextFrameSetupSystem2,
                                       Rectangle, RigidBody, SpatialCollisionSystem2,
                                       SpatialSortingSystem2, WithRigidBody};

struct RayCastSystem;

impl<'a> System<'a> for RayCastSystem {
    type SystemData = (Fetch<'a, DynamicBoundingVolumeTree2>,);

    fn run(&mut self, (tree,): Self::SystemData) {
        let ray = Ray2::new(Point2::new(-4., 10.), Vector2::new(0., -1.));
        if let Some((v, p)) = query_ray_closest(&*tree, ray) {
            println!("hit bounding volume of {:?} at point {:?}", v.value, p);
        }
    }
}

pub fn main() {
    let mut world = World::new();
    register_physics::<()>(&mut world);

    world
        .create_entity()
        .with_static_rigid_body(
            CollisionShape2::<BodyPose2, ()>::new_simple(
                CollisionStrategy::FullResolution,
                CollisionMode::Discrete,
                Rectangle::new(10., 10.).into(),
            ),
            BodyPose2::one(),
            RigidBody::default(),
            Mass2::new(1.),
        )
        .build();

    world
        .create_entity()
        .with_static_rigid_body(
            CollisionShape2::<BodyPose2, ()>::new_simple(
                CollisionStrategy::FullResolution,
                CollisionMode::Discrete,
                Rectangle::new(10., 10.).into(),
            ),
            BodyPose2::new(Point2::new(2., 2.), Rotation2::from_angle(Rad(0.))),
            RigidBody::default(),
            Mass2::new(1.),
        )
        .build();

    let mut reader_1 = world
        .write_resource::<EventChannel<ContactEvent2>>()
        .register_reader();
    let reader_2 = world
        .write_resource::<EventChannel<ContactEvent2>>()
        .register_reader();

    let mut sort = SpatialSortingSystem2::<BodyPose2, ()>::new();
    let mut collide =
        SpatialCollisionSystem2::<BodyPose2, ()>::new().with_narrow_phase(GJK2::new());
    let mut raycast = RayCastSystem;
    sort.run_now(&world.res);
    collide.run_now(&world.res);
    println!(
        "Contacts: {:?}",
        world
            .read_resource::<EventChannel<ContactEvent2>>()
            .read(&mut reader_1)
            .collect::<Vec<_>>()
    );
    raycast.run_now(&world.res);

    let mut impulse_solver = CurrentFrameUpdateSystem2::new();
    impulse_solver.run_now(&world.res);
    let mut next_frame = NextFrameSetupSystem2::new();
    next_frame.run_now(&world.res);
    let mut contact_resolution = ContactResolutionSystem2::new(reader_2);
    contact_resolution.run_now(&world.res);
}
