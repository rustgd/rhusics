Physics library for use in Specs, using cgmath and collision-rs.

# Example

```rust
use cgmath::{Transform, Rotation2, Rad, Point2};
use specs::{World, RunNow};

use collide2d::*;

type Shape = CollisionShape2D<BodyPose2D>;
type Pose = BodyPose2D;
type System = CollisionSystem2D<Pose>;

#[test]
pub fn test_world() {
    let mut world = World::new();
    world_register::<Pose>(&mut world);
    world
        .create_entity()
        .with(Shape::new_simple(CollisionStrategy::FullResolution, Rectangle::new(10., 10.).into(),))
        .with(Pose::one());
    world
        .create_entity()
        .with(Shape::new_simple(CollisionStrategy::FullResolution, Rectangle::new(10., 10.).into(),))
        .with(Pose::new(Point2::new(3., 2.), Rotation2::from_angle(Rad(0.))));

    let mut system = System::new()
        .with_broad_phase(BroadBruteForce2D::default())
        .with_narrow_phase(GJK2D::new());
    system.run_now(&world.res);
    println!("Contacts: {:?}", *world.read_resource::<Contacts2D>());
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
    * Box
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

# TODO:

* Improved sweep and prune
* Dynamic Bounding Volume Tree
* Spatial sorting to the CollisionSystem
* Raycasting + picking
* Broad phase using DBVT
* Contact manifold with contact points
* Impulse solver with both equality and non-equality constraints
* Better primitive type for convex polytope (adjacent edge)
