## Changelog

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
