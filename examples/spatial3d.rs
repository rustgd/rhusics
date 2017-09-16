extern crate rhusics;
extern crate cgmath;
extern crate specs;
extern crate collision;

use cgmath::{Transform, Rotation3, Rad, Point3, Quaternion, Vector3};
use collision::Ray3;
use collision::dbvt::query_ray_closest;
use specs::{World, RunNow, System, Fetch};

use rhusics::collide::broad::BroadCollisionData;
use rhusics::collide3d::{CollisionShape3, SpatialSortingSystem3, SpatialCollisionSystem3,
                         BodyPose3, DynamicBoundingVolumeTree3, GJK3, world_register_with_spatial,
                         Cuboid, Contacts3, CollisionStrategy};

struct RayCastSystem;

impl<'a> System<'a> for RayCastSystem {
    type SystemData = (Fetch<'a, DynamicBoundingVolumeTree3>,);

    fn run(&mut self, (tree,): Self::SystemData) {
        let ray = Ray3::new(Point3::new(-4., 10., 0.), Vector3::new(0., -1., 0.));
        if let Some((v, p)) = query_ray_closest(&*tree, ray) {
            println!("hit bounding volume of {:?} at point {:?}", v.id(), p);
        }
    }
}

pub fn main() {
    let mut world = World::new();
    world_register_with_spatial::<BodyPose3>(&mut world);

    world
        .create_entity()
        .with(CollisionShape3::<BodyPose3>::new_simple(
            CollisionStrategy::FullResolution,
            Cuboid::new(10., 10., 10.).into(),
        ))
        .with(BodyPose3::one());

    world
        .create_entity()
        .with(CollisionShape3::<BodyPose3>::new_simple(
            CollisionStrategy::FullResolution,
            Cuboid::new(10., 10., 10.).into(),
        ))
        .with(BodyPose3::new(
            Point3::new(3., 2., 0.),
            Quaternion::from_angle_z(Rad(0.)),
        ));

    let mut sort = SpatialSortingSystem3::<BodyPose3>::new();
    let mut collide = SpatialCollisionSystem3::<BodyPose3>::new().with_narrow_phase(GJK3::new());
    let mut raycast = RayCastSystem;
    sort.run_now(&world.res);
    collide.run_now(&world.res);
    println!("Contacts: {:?}", *world.read_resource::<Contacts3>());
    raycast.run_now(&world.res);
}
