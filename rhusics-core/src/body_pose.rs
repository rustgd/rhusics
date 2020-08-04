use cgmath::{BaseFloat, EuclideanSpace, InnerSpace, One, Rotation, Transform, VectorSpace};
use collision::{Interpolate, TranslationInterpolate};

use Pose;

/// Transform component used throughout the library
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(target_feature = "serializable", derive(Serialize, Deserialize))]
pub struct BodyPose<P, R> {
    dirty: bool,
    position: P,
    rotation: R,
    inverse_rotation: R,
}

impl<P, R> Pose<P, R> for BodyPose<P, R>
where
    P: EuclideanSpace,
    P::Scalar: BaseFloat,
    R: Rotation<P>,
{
    /// Create a new [`BodyPose`](struct.BodyPose.html) with initial state given by the supplied
    /// position and rotation.
    fn new(position: P, rotation: R) -> Self {
        Self {
            dirty: true,
            position,
            inverse_rotation: rotation.invert(),
            rotation,
        }
    }

    /// Set the rotation. Will also compute the inverse rotation. Sets the dirty flag.
    fn set_rotation(&mut self, rotation: R) {
        self.rotation = rotation;
        self.inverse_rotation = self.rotation.invert();
        self.dirty = true;
    }

    /// Set the position. Sets the dirty flag.
    fn set_position(&mut self, position: P) {
        self.position = position;
        self.dirty = true;
    }

    /// Borrows the rotation attribute
    fn rotation(&self) -> R {
        self.rotation
    }

    /// Borrows the position attribute
    fn position(&self) -> P {
        self.position
    }
}

impl<P, R> BodyPose<P, R>
where
    P: EuclideanSpace,
    P::Scalar: BaseFloat,
    R: Rotation<P>,
{
    /// Clear the dirty flag
    pub fn clear(&mut self) {
        self.dirty = false;
    }
}

impl<P, R> Transform<P> for BodyPose<P, R>
where
    P: EuclideanSpace,
    P::Scalar: BaseFloat,
    R: Rotation<P>,
{
    fn one() -> Self {
        Self::new(P::origin(), R::one())
    }

    fn look_at(eye: P, center: P, up: P::Diff) -> Self {
        let rot = R::look_at(center - eye, up);
        let disp = rot.rotate_vector(P::origin() - eye);
        Self::new(P::from_vec(disp), rot)
    }

    fn transform_vector(&self, vec: P::Diff) -> P::Diff {
        self.rotation.rotate_vector(vec)
    }

    fn inverse_transform_vector(&self, vec: P::Diff) -> Option<P::Diff> {
        Some(self.inverse_rotation.rotate_vector(vec))
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
            self.rotation.rotate_point(self.position) * -P::Scalar::one(),
            self.inverse_rotation,
        ))
    }
}

impl<P, R> TranslationInterpolate<P::Scalar> for BodyPose<P, R>
where
    P: EuclideanSpace,
    P::Scalar: BaseFloat,
    P::Diff: VectorSpace + InnerSpace,
    R: Rotation<P> + Clone,
{
    fn translation_interpolate(&self, other: &Self, amount: P::Scalar) -> Self {
        BodyPose::new(
            P::from_vec(self.position.to_vec().lerp(other.position.to_vec(), amount)),
            other.rotation,
        )
    }
}

impl<P, R> Interpolate<P::Scalar> for BodyPose<P, R>
where
    P: EuclideanSpace,
    P::Scalar: BaseFloat,
    P::Diff: VectorSpace + InnerSpace,
    R: Rotation<P> + Interpolate<P::Scalar>,
{
    fn interpolate(&self, other: &Self, amount: P::Scalar) -> Self {
        BodyPose::new(
            P::from_vec(self.position.to_vec().lerp(other.position.to_vec(), amount)),
            self.rotation.interpolate(&other.rotation, amount),
        )
    }
}
