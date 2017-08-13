pub mod collision;

use cgmath::{Vector2, Zero, Point2, Array, Rotation2, Rad, Matrix2, Basis2, Matrix};
use collision::Aabb2;

#[derive(Debug)]
pub struct BodyPose {
    pub dirty : bool,
    pub position: Point2<f32>,
    pub rotation: Matrix2<f32>,
    pub inverse_rotation: Matrix2<f32>
}

#[derive(Debug)]
pub struct Spatial {
    pub current : BodyPose,
    pub next : BodyPose,
}

impl Default for BodyPose {
    fn default() -> Self {
        let rot: Basis2<f32> = Rotation2::from_angle(Rad(0.));
        BodyPose {
            dirty: false,
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

#[derive(Debug)]
pub struct BodyMass {
    pub density : f32,
    pub volume : f32,
    pub inertia_tensor : f32
}

impl Default for BodyMass {
    fn default() -> Self {
        BodyMass {
            density : 0.,
            volume : 0.,
            inertia_tensor : 0.
        }
    }
}

impl BodyMass {
    pub fn mass(&self) -> f32 {
        self.volume * self.density
    }
}

#[derive(Debug)]
pub struct Velocity {
    pub linear : Vector2<f32>,
    pub angular : Vector2<f32>
}

impl Default for Velocity {
    fn default() -> Velocity {
        Velocity {
            linear : Vector2::zero(),
            angular : Vector2::zero()
        }
    }
}

#[derive(Debug)]
pub struct Force {
    pub linear : Vector2<f32>,
    pub angular : Vector2<f32>
}

impl Default for Force {
    fn default() -> Self {
        Force {
            linear : Vector2::zero(),
            angular : Vector2::zero(),
        }
    }
}

#[derive(Debug)]
pub struct Body {
    pub enabled : bool,
    pub restitution : f32,
    pub velocity : Velocity,
    pub mass : BodyMass,
    pub force : Force
}

impl Default for Body {
    fn default() -> Body {
        Body {
            enabled : true,
            restitution : 0.,
            velocity : Velocity::default(),
            mass : BodyMass::default(),
            force : Force::default(),
        }
    }
}

#[derive(Component, Debug)]
pub struct Shape {
    pub enabled : bool,
    pub bounding : Aabb2<f32>,
    pub offset : Vector2<f32>,
    pub primitives : Vec<self::collision::CollisionPrimitive>
}

impl Default for Shape {
    fn default() -> Shape {
        Shape {
            enabled : false,
            offset : Vector2::zero(),
            bounding : Aabb2::new(Point2::from_value(0.), Point2::from_value(0.)),
            primitives : Vec::default(),
        }
    }
}
