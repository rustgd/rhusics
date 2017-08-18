extern crate cgmath;
extern crate collision;
extern crate specs;

#[macro_use]
extern crate log;

#[macro_use]
extern crate specs_derive;

pub mod util;
pub mod experiment;
pub mod collide;

use cgmath::{BaseFloat, VectorSpace, ElementWise, Array, EuclideanSpace, Rotation, Decomposed};

#[derive(Clone)]
pub struct BodyPose<S, V, P, R>
where
    S: BaseFloat + Clone,
    V: VectorSpace<Scalar = S> + ElementWise + Array<Element = S>,
    P: EuclideanSpace<Scalar = S, Diff = V>,
    R: Rotation<P>,
{
    pub dirty: bool,
    pub position: P,
    pub rotation: R,
    pub inverse_rotation: R,
}

impl<S, V, P, R> Into<Decomposed<V, R>> for BodyPose<S, V, P, R>
where
    S: BaseFloat,
    V: VectorSpace<Scalar = S>
        + ElementWise
        + Array<Element = S>,
    P: EuclideanSpace<Scalar = S, Diff = V>,
    R: Rotation<P>,
{
    fn into(self) -> Decomposed<V, R> {
        Decomposed {
            scale : S::one(),
            rot : self.rotation.clone(),
            disp : self.position.to_vec()
        }
    }
}
