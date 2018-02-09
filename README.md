Physics library for use in Specs, using cgmath and collision-rs.

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
* Has support for doing spatial sort/collision detection using the collision-rs DBVT.
* Support for doing broad phase using the collision-rs DBVT.
* Continuous collision detection, using GJK
* Simple rigid body implementation with single contact forward resolution

# TODO:

* Impulse solver
* Integrator implementations (Euler, RK4, etc.) 
* Parallel solver implementation

## License

Licensed under either of

 * Apache License, Version 2.0, ([LICENSE-APACHE](LICENSE-APACHE) or http://www.apache.org/licenses/LICENSE-2.0)
 * MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.

### Contribution

We are a community project that welcomes contribution from anyone. If you're interested in helping out, you can contact
us either through GitHub, or via [`gitter`](https://gitter.im/collision-rs/Lobby).

Unless you explicitly state otherwise, any contribution intentionally submitted
for inclusion in the work by you, as defined in the Apache-2.0 license, shall be dual licensed as above, without any
additional terms or conditions.
