pub mod collision;
pub mod impulse;

use cgmath::{Point2, Array, Rotation2, Rad, Matrix2, Basis2, Matrix};

#[derive(Debug, Component)]
pub struct BodyPose {
    pub dirty: bool,
    pub position: Point2<f32>,
    pub rotation: Matrix2<f32>,
    pub inverse_rotation: Matrix2<f32>,
}

impl Default for BodyPose {
    fn default() -> Self {
        let rot: Basis2<f32> = Rotation2::from_angle(Rad(0.));
        BodyPose {
            dirty: true,
            position: Point2::from_value(0.),
            rotation: rot.into(),
            inverse_rotation: rot.into(),
        }
    }
}

impl BodyPose {
    pub fn set_rotation(&mut self, rotation: Matrix2<f32>) {
        self.dirty = true;
        self.rotation = rotation;
        self.inverse_rotation = self.rotation.transpose();
    }

    pub fn set_position(&mut self, position: Point2<f32>) {
        self.position = position;
        self.dirty = true;
    }

    pub fn clear_dirty_flag(&mut self) {
        self.dirty = false;
    }
}
