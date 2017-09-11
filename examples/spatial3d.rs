extern crate rhusics;
extern crate cgmath;
extern crate specs;
extern crate collision;

use cgmath::{Transform, Rotation3, Rad, Point3, Quaternion, Vector3};
use collision::Ray3;
use specs::{World, RunNow, System, Entity, Fetch};

use rhusics::collide3d::{CollisionShape3D, SpatialSortingSystem3D, SpatialCollisionSystem3D,
                         BodyPose3D, BroadBruteForce3D, DynamicBoundingVolumeTree3D, GJK3D,
                         world_register_with_spatial, Cuboid, Contacts3D, CollisionStrategy};

struct RayCastSystem;

impl<'a> System<'a> for RayCastSystem {
    type SystemData = (Fetch<'a, DynamicBoundingVolumeTree3D<Entity>>,);

    fn run(&mut self, (tree,): Self::SystemData) {
        let ray = Ray3::new(Point3::new(-4., 10., 0.), Vector3::new(0., -1., 0.));
        println!("{:?}", tree.query_ray_closest(&ray));
    }
}

pub fn main() {
    let mut world = World::new();
    world_register_with_spatial::<BodyPose3D>(&mut world);

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

    let mut sort = SpatialSortingSystem3D::<BodyPose3D>::new();
    let mut collide = SpatialCollisionSystem3D::<BodyPose3D>::new()
        .with_broad_phase(BroadBruteForce3D::default())
        .with_narrow_phase(GJK3D::new());
    let mut raycast = RayCastSystem;
    sort.run_now(&world.res);
    collide.run_now(&world.res);
    println!("Contacts: {:?}", *world.read_resource::<Contacts3D>());
    raycast.run_now(&world.res);
}
