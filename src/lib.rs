extern crate cgmath;
extern crate collision;
extern crate specs;

#[macro_use]
extern crate log;

pub mod util;
pub mod collide;
pub mod collide2d;
pub mod ecs;

use cgmath::prelude::*;
use cgmath::{BaseFloat, Decomposed};

#[derive(Clone, Debug)]
pub struct BodyPose<P, R>
where
    P: EuclideanSpace,
    P::Scalar: BaseFloat,
    R: Rotation<P>,
{
    pub dirty: bool,
    pub position: P,
    pub rotation: R,
    pub inverse_rotation: R,
}

impl<P, R> BodyPose<P, R>
where
    P: EuclideanSpace,
    P::Scalar: BaseFloat,
    R: Rotation<P>,
{
    pub fn new(position: P, rotation: R) -> Self {
        Self {
            dirty: true,
            position,
            inverse_rotation: rotation.invert(),
            rotation,
        }
    }

    pub fn zero() -> Self {
        Self::new(P::from_value(P::Scalar::zero()), R::one())
    }
}

impl<'a, P, R> Into<Decomposed<P::Diff, R>> for &'a BodyPose<P, R>
where
    P: EuclideanSpace,
    P::Scalar: BaseFloat,
    R: Rotation<P>,
{
    fn into(self) -> Decomposed<P::Diff, R> {
        Decomposed {
            scale: P::Scalar::one(),
            rot: self.rotation.clone(),
            disp: self.position.to_vec(),
        }
    }
}

#[cfg(test)]
mod tests {
    use collide2d::*;
    use specs::{World, RunNow};
    use std::ops::Deref;

    #[test]
    pub fn test_world() {
        let mut world = World::new();
        world.register::<CollisionShape2D<f32>>();
        world.register::<BodyPose2D<f32>>();
        world.add_resource(Contacts2D::<f32>::new());
        world
            .create_entity()
            .with(CollisionShape2D::<f32>::new_simple(
                CollisionStrategy::CollisionOnly,
                Rectangle::new(10., 10.).into(),
            ))
            .with(BodyPose2D::<f32>::zero());
        let mut system =
            CollisionSystem2D::<f32>::new().with_broad_phase(BroadBruteForce2D::default());
        system.run_now(&world.res);
        println!(
            "Contacts: {:?}",
            world.read_resource::<Contacts2D<f32>>().deref()
        );
    }
}
