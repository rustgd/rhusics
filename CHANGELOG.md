## Changelog

### v0.9 [0.5 for rhusics-transform]
- updated shred to 0.10.
- updated specs to 0.16, modified to new API ("World" instead of "Resources").
- updated boxed class refs to include dyn, as requested by compiler.
- fixed bug in toml files where was always using serde.
- BREAKING CHANGE: use "serializable" instead of "serde" to get serialization.
  Using just "serde" won't compile any more.

### v0.8 [0.5 for rhusics-transform]
- Update cgmath to 0.17
- Update collision to 0.20
- Update approx to 0.3
- No API changes, but recompilation necessary as supporting traits changed.

### v0.7

- Update specs version to 0.14

### v0.4

- Update to new version of collision:
  * BREAKING CHANGE: Primitive2 and Primitive3 have new variants
  * BREAKING CHANGE: Signature change of `GJK::intersect`
- refactor: Rename feature `eders` to `serde`
- fix: Crashed if a collision was detected, but the Entity was removed before collision resolution
- feat: Implement Pose for cgmath::Decomposed
- feat: Abstract the time ECS resource, to make it pluggable, BREAKING CHANGE
- refactor: Adapt ECS integration to new Specs, BREAKING CHANGE

### v0.3

- Core code is now generic wrt transform type
- Trait required to implement for the transform type is broken out to a separate crate `rhusics-transform`

### v0.2

- Update to new version of collision
  * BREAKING CHANGE: Primitive2 and Primitive3 have new variants
