extern crate cgmath;
extern crate collision;
extern crate rhusics;
extern crate shrev;
extern crate specs;

use cgmath::{Point3, Quaternion, Rad, Rotation3, Transform, Vector3};
use collision::Ray3;
use collision::dbvt::query_ray_closest;
use shrev::EventChannel;
use specs::{Fetch, RunNow, System, World};

use rhusics::ecs::physics::prelude3d::{register_physics, BodyPose3, CollisionMode,
                                       CollisionShape3, CollisionStrategy, ContactEvent3,
                                       ContactResolutionSystem3, Cuboid,
                                       DynamicBoundingVolumeTree3, GJK3, CurrentFrameUpdateSystem3,
                                       Mass3, NextFrameSetupSystem3, RigidBody,
                                       SpatialCollisionSystem3, SpatialSortingSystem3,
                                       WithRigidBody};

struct RayCastSystem;

impl<'a> System<'a> for RayCastSystem {
    type SystemData = (Fetch<'a, DynamicBoundingVolumeTree3<f32>>,);

    fn run(&mut self, (tree,): Self::SystemData) {
        let ray = Ray3::new(Point3::new(-4., 10., 0.), Vector3::new(0., -1., 0.));
        if let Some((v, p)) = query_ray_closest(&*tree, ray) {
            println!("hit bounding volume of {:?} at point {:?}", v.value, p);
        }
    }
}

pub fn main() {
    let mut world = World::new();
    register_physics::<f32, ()>(&mut world);

    world
        .create_entity()
        .with_static_rigid_body(
            CollisionShape3::<f32, BodyPose3<f32>, ()>::new_simple(
                CollisionStrategy::FullResolution,
                CollisionMode::Discrete,
                Cuboid::new(10., 10., 10.).into(),
            ),
            BodyPose3::one(),
            RigidBody::default(),
            Mass3::new(1.),
        )
        .build();

    world
        .create_entity()
        .with_static_rigid_body(
            CollisionShape3::<f32, BodyPose3<f32>, ()>::new_simple(
                CollisionStrategy::FullResolution,
                CollisionMode::Discrete,
                Cuboid::new(10., 10., 10.).into(),
            ),
            BodyPose3::new(Point3::new(3., 2., 0.), Quaternion::from_angle_z(Rad(0.))),
            RigidBody::default(),
            Mass3::new(1.),
        )
        .build();

    let mut reader_1 = world
        .write_resource::<EventChannel<ContactEvent3<f32>>>()
        .register_reader();
    let reader_2 = world
        .write_resource::<EventChannel<ContactEvent3<f32>>>()
        .register_reader();

    let mut sort = SpatialSortingSystem3::<f32, BodyPose3<f32>, ()>::new();
    let mut collide =
        SpatialCollisionSystem3::<f32, BodyPose3<f32>, ()>::new().with_narrow_phase(GJK3::new());
    let mut raycast = RayCastSystem;
    sort.run_now(&world.res);
    collide.run_now(&world.res);
    println!(
        "Contacts: {:?}",
        world
            .read_resource::<EventChannel<ContactEvent3<f32>>>()
            .read(&mut reader_1)
            .collect::<Vec<_>>()
    );
    raycast.run_now(&world.res);

    let mut impulse_solver = CurrentFrameUpdateSystem3::<f32>::new();
    impulse_solver.run_now(&world.res);
    let mut next_frame = NextFrameSetupSystem3::<f32>::new();
    next_frame.run_now(&world.res);
    let mut contact_resolution = ContactResolutionSystem3::<f32>::new(reader_2);
    contact_resolution.run_now(&world.res);
}
