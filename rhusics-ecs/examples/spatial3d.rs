extern crate cgmath;
extern crate collision;
extern crate rhusics_core;
extern crate rhusics_ecs;
extern crate shrev;
extern crate specs;

use cgmath::{Point3, Quaternion, Rad, Rotation3, Transform, Vector3};
use collision::dbvt::query_ray_closest;
use collision::Ray3;
use shrev::EventChannel;
use specs::prelude::{Builder, ReadExpect, System, World, WorldExt};

use rhusics_core::{PhysicalEntity, Pose};
use rhusics_ecs::physics3d::{
    BodyPose3, CollisionMode, CollisionShape3, CollisionStrategy, ContactEvent3,
    ContactResolutionSystem3, Cuboid, CurrentFrameUpdateSystem3, DynamicBoundingVolumeTree3, Mass3,
    NextFrameSetupSystem3, SpatialCollisionSystem3, SpatialSortingSystem3, GJK3,
};
use rhusics_ecs::WithPhysics;

struct RayCastSystem;

impl<'a> System<'a> for RayCastSystem {
    type SystemData = (ReadExpect<'a, DynamicBoundingVolumeTree3<f32>>,);

    fn run(&mut self, (tree,): Self::SystemData) {
        let ray = Ray3::new(Point3::new(-4., 10., 0.), Vector3::new(0., -1., 0.));
        if let Some((v, p)) = query_ray_closest(&*tree, ray) {
            println!("hit bounding volume of {:?} at point {:?}", v.value, p);
        }
    }
}

pub fn main() {
    let mut world = World::new();
    let mut sort = SpatialSortingSystem3::<f32, BodyPose3<f32>, ()>::new();
    let mut collide =
        SpatialCollisionSystem3::<f32, BodyPose3<f32>, ()>::new().with_narrow_phase(GJK3::new());
    let mut raycast = RayCastSystem;
    let mut impulse_solver = CurrentFrameUpdateSystem3::<f32, BodyPose3<f32>>::new();
    let mut next_frame = NextFrameSetupSystem3::<f32, BodyPose3<f32>>::new();
    let mut contact_resolution = ContactResolutionSystem3::<f32, BodyPose3<f32>>::new();

    sort.setup(&mut world);
    collide.setup(&mut world);
    raycast.setup(&mut world);
    impulse_solver.setup(&mut world);
    next_frame.setup(&mut world);
    contact_resolution.setup(&mut world);

    world
        .create_entity()
        .with_static_physical_entity(
            CollisionShape3::<f32, BodyPose3<f32>, ()>::new_simple(
                CollisionStrategy::FullResolution,
                CollisionMode::Discrete,
                Cuboid::new(10., 10., 10.).into(),
            ),
            BodyPose3::one(),
            PhysicalEntity::default(),
            Mass3::new(1.),
        ).build();

    world
        .create_entity()
        .with_static_physical_entity(
            CollisionShape3::<f32, BodyPose3<f32>, ()>::new_simple(
                CollisionStrategy::FullResolution,
                CollisionMode::Discrete,
                Cuboid::new(10., 10., 10.).into(),
            ),
            BodyPose3::new(Point3::new(3., 2., 0.), Quaternion::from_angle_z(Rad(0.))),
            PhysicalEntity::default(),
            Mass3::new(1.),
        ).build();

    let mut reader_1 = world
        .write_resource::<EventChannel<ContactEvent3<f32>>>()
        .register_reader();
    {
        use specs::prelude::RunNow;
        sort.run_now(&world);
        collide.run_now(&world);
        println!(
            "Contacts: {:?}",
            world
                .read_resource::<EventChannel<ContactEvent3<f32>>>()
                .read(&mut reader_1)
                .collect::<Vec<_>>()
        );
        raycast.run_now(&world);

        impulse_solver.run_now(&world);
        next_frame.run_now(&world);
        contact_resolution.run_now(&world);
    }
}
