extern crate cgmath;
extern crate collision;
extern crate specs;

#[macro_use]
extern crate log;

#[cfg(test)]
#[macro_use]
extern crate assert_approx_eq;

pub mod util;
pub mod collide;
pub mod collide2d;
pub mod collide3d;
pub mod ecs;

use cgmath::prelude::*;

#[cfg(not(feature = "double"))]
pub(crate) type Real = f32;

#[cfg(feature = "double")]
pub(crate) type Real = f64;

pub trait Pose<P>: Transform<P>
where
    P: EuclideanSpace<Scalar = Real>,
{
    type Rotation: Rotation<P>;

    fn position(&self) -> &P;
    fn rotation(&self) -> &Self::Rotation;
    fn dirty(&self) -> bool;
    fn inverse_rotation(&self) -> &Self::Rotation;
}

#[derive(Clone, Debug)]
pub struct BodyPose<P, R>
where
    P: EuclideanSpace<Scalar = Real>,
    R: Rotation<P>,
{
    pub dirty: bool,
    pub position: P,
    pub rotation: R,
    pub inverse_rotation: R,
}

impl<P, R> BodyPose<P, R>
where
    P: EuclideanSpace<Scalar = Real>,
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
}

impl<P, R> Transform<P> for BodyPose<P, R>
where
    P: EuclideanSpace<Scalar = Real>,
    R: Rotation<P>,
{
    fn one() -> Self {
        Self::new(P::from_value(0.), R::one())
    }

    fn look_at(eye: P, center: P, up: P::Diff) -> Self {
        let rot = R::look_at(center - eye, up);
        let disp = rot.rotate_vector(P::origin() - eye);
        Self::new(P::from_vec(disp), rot)
    }

    fn transform_vector(&self, vec: P::Diff) -> P::Diff {
        self.rotation.rotate_vector(vec)
    }

    fn transform_point(&self, point: P) -> P {
        self.rotation.rotate_point(point) + self.position.to_vec()
    }

    fn concat(&self, other: &Self) -> Self {
        Self::new(
            self.position + self.rotation.rotate_point(other.position).to_vec(),
            self.rotation * other.rotation,
        )
    }

    fn inverse_transform(&self) -> Option<Self> {
        Some(Self::new(
            self.rotation.rotate_point(self.position) *
                -P::Scalar::one(),
            self.inverse_rotation,
        ))
    }
}

impl<P, R> Pose<P> for BodyPose<P, R>
where
    P: EuclideanSpace<Scalar = Real>,
    R: Rotation<P>,
{
    type Rotation = R;

    fn position(&self) -> &P {
        &self.position
    }

    fn rotation(&self) -> &R {
        &self.rotation
    }

    fn dirty(&self) -> bool {
        self.dirty
    }

    fn inverse_rotation(&self) -> &R {
        &self.inverse_rotation
    }
}
