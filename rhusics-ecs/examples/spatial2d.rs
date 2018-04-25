extern crate cgmath;
extern crate collision;
extern crate rhusics_core;
extern crate rhusics_ecs;
extern crate shrev;
extern crate specs;

use cgmath::{Point2, Rad, Rotation2, Transform, Vector2};
use collision::Ray2;
use collision::dbvt::query_ray_closest;
use shrev::EventChannel;
use specs::prelude::{ReadExpect, System, World};

use rhusics_core::{Pose, RigidBody};
use rhusics_ecs::WithRigidBody;
use rhusics_ecs::physics2d::{BodyPose2, CollisionMode, CollisionShape2, CollisionStrategy,
                             ContactEvent2, ContactResolutionSystem2, CurrentFrameUpdateSystem2,
                             DynamicBoundingVolumeTree2, GJK2, Mass2, NextFrameSetupSystem2,
                             Rectangle, SpatialCollisionSystem2, SpatialSortingSystem2};

struct RayCastSystem;

impl<'a> System<'a> for RayCastSystem {
    type SystemData = (ReadExpect<'a, DynamicBoundingVolumeTree2<f32>>,);

    fn run(&mut self, (tree,): Self::SystemData) {
        println!("{:?}", *tree);
        let ray = Ray2::new(Point2::new(-4., 10.), Vector2::new(0., -1.));
        if let Some((v, p)) = query_ray_closest(&*tree, ray) {
            println!("hit bounding volume of {:?} at point {:?}", v.value, p);
        }
    }
}

pub fn main() {
    let mut world = World::new();
    let mut sort = SpatialSortingSystem2::<f32, BodyPose2<f32>, ()>::new();
    let mut collide =
        SpatialCollisionSystem2::<f32, BodyPose2<f32>, ()>::new().with_narrow_phase(GJK2::new());
    let mut raycast = RayCastSystem;
    let mut impulse_solver = CurrentFrameUpdateSystem2::<f32, BodyPose2<f32>>::new();
    let mut next_frame = NextFrameSetupSystem2::<f32, BodyPose2<f32>>::new();
    let mut contact_resolution = ContactResolutionSystem2::<f32, BodyPose2<f32>>::new();

    sort.setup(&mut world.res);
    collide.setup(&mut world.res);
    raycast.setup(&mut world.res);
    impulse_solver.setup(&mut world.res);
    next_frame.setup(&mut world.res);
    contact_resolution.setup(&mut world.res);

    world
        .create_entity()
        .with_static_rigid_body(
            CollisionShape2::<f32, BodyPose2<f32>, ()>::new_simple(
                CollisionStrategy::FullResolution,
                CollisionMode::Discrete,
                Rectangle::new(10., 10.).into(),
            ),
            BodyPose2::<f32>::one(),
            RigidBody::default(),
            Mass2::new(1.),
        )
        .build();

    world
        .create_entity()
        .with_static_rigid_body(
            CollisionShape2::<f32, BodyPose2<f32>, ()>::new_simple(
                CollisionStrategy::FullResolution,
                CollisionMode::Discrete,
                Rectangle::new(10., 10.).into(),
            ),
            BodyPose2::<f32>::new(Point2::new(2., 2.), Rotation2::from_angle(Rad(0.))),
            RigidBody::default(),
            Mass2::new(1.),
        )
        .build();

    let mut reader_1 = world
        .write_resource::<EventChannel<ContactEvent2<f32>>>()
        .register_reader();

    {
        use specs::prelude::RunNow;
        sort.run_now(&world.res);
        collide.run_now(&world.res);

        println!(
            "Contacts: {:?}",
            world
                .read_resource::<EventChannel<ContactEvent2<f32>>>()
                .read(&mut reader_1)
                .collect::<Vec<_>>()
        );

        raycast.run_now(&world.res);
        impulse_solver.run_now(&world.res);
        next_frame.run_now(&world.res);
        contact_resolution.run_now(&world.res);
    }
}
