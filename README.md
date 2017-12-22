Physics library for use in Specs, using cgmath and collision-rs.

# Example

```rust
extern crate cgmath;
extern crate rhusics;
extern crate specs;

use cgmath::{Point2, Rad, Rotation2, Transform};
use specs::{RunNow, World};

use rhusics::collide2d::{world_register, BasicCollisionSystem2, BodyPose2, BroadBruteForce2,
                         CollisionMode, CollisionShape2, CollisionStrategy, Contacts2, GJK2,
                         Rectangle};

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
```

# Features:

* Has support for all primitives in collision-rs
* Has support for the following broad phase algorithms in collision-rs:
  * Brute force
  * Sweep and Prune
* Narrow phase collision detection using GJK, and optionally EPA for full contact information
* [`specs::System`](https://docs.rs/specs/0.9.5/specs/trait.System.html) for collision
  detection working on user supplied transform, and shape components.
  Can optionally use broad and/or narrow phase detection.
  Library supplies a transform implementation for convenience.
* [`specs::System`](https://docs.rs/specs/0.9.5/specs/trait.System.html) for spatial
  sorting on user supplied transform, and shape components.
* Uses single precision as default, can be changed to double precision with the `double`
  feature.
* Has support for doing spatial sort/collision detection using the collision-rs DBVT.
* Support for doing broad phase using the collision-rs DBVT.
* Continuous collision detection, using GJK

# TODO:

* Impulse solver
* Integrator implementations (Euler, RK4, etc.) 
* Inertial moments for bodies at rest
* Parallel solver implementation
* Generic bounding volumes
