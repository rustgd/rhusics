Physics library for use in Specs, using cgmath and collision-rs.

# Example

```rust
extern crate rhusics;
extern crate cgmath;
extern crate specs;

use cgmath::{Transform, Rotation2, Rad, Point2};
use specs::{World, RunNow};

use rhusics::collide2d::{CollisionShape2, BasicCollisionSystem2, BodyPose2,
                      BroadBruteForce2, GJK2, world_register, Rectangle,
                      Contacts2, CollisionStrategy};

pub fn main() {
    let mut world = World::new();
    world_register::<BodyPose2>(&mut world);
    
    world
    .create_entity()
    .with(CollisionShape2::<BodyPose2>::new_simple(
        CollisionStrategy::FullResolution,
        Rectangle::new(10., 10.).into(),
    ))
    .with(BodyPose2::one());
    
    world
        .create_entity()
        .with(CollisionShape2::<BodyPose2>::new_simple(
            CollisionStrategy::FullResolution,
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
```

# Features:

* Collision detection primitives for 2D and 3D:
  * 2D
    * Circle
    * Rectangle
    * Convex polygon with any number of vertices
  * 3D
    * Sphere
    * Cuboid
    * Convex polytope with any number of vertices
* Two different broad phase collision detection implementations:
  * Brute force
  * Sweep and Prune
* Narrow phase collision detection using GJK, and optionally EPA for full contact information
* [`specs::System`](https://docs.rs/specs/0.9.5/specs/trait.System.html) for collision
  detection working on user supplied transform, and shape components.
  Can optionally use broad and/or narrow phase detection.
  Library supplies a transform implementation for convenience.
* Uses single precision as default, can be changed to double precision with the `double`
  feature.
* Has support for spatial sorting using a dynamic bounding volume tree.
* Can perform continuous, discrete, frustum queries on the DBVT
* Can perform custom queries on the DBVT
* Support for doing broad phase using DBVT.

# TODO:

* Contact manifold with contact points
* Better primitive type for convex polytope (half edge)
* Continuous collision detection
* Ray intersection tests with actual geometry
* Impulse solver
