use cgmath::{Vector2, Zero};

use std::ops::{Add, AddAssign};

#[derive(Debug)]
pub struct BodyMass {
    pub density: f32,
    pub volume: f32,
    pub inertia_tensor: f32,
}

impl Default for BodyMass {
    fn default() -> Self {
        BodyMass {
            density: 0.,
            volume: 0.,
            inertia_tensor: 0.,
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
    pub linear: Vector2<f32>,
    pub angular: Vector2<f32>,
}

impl Default for Velocity {
    fn default() -> Velocity {
        Velocity {
            linear: Vector2::zero(),
            angular: Vector2::zero(),
        }
    }
}

#[derive(Debug)]
pub struct Impulse {
    pub linear: Vector2<f32>,
    pub angular: Vector2<f32>,
}

impl Impulse {
    pub fn new(linear: Vector2<f32>, angular: Vector2<f32>) -> Self {
        Impulse { linear, angular }
    }

    pub fn zero(&mut self) {
        *self = Impulse::default();
    }
}
impl Default for Impulse {
    fn default() -> Self {
        Impulse::new(Vector2::zero(), Vector2::zero())
    }
}

impl Add for Impulse {
    type Output = Impulse;

    fn add(self, other: Impulse) -> Impulse {
        Impulse {
            linear: self.linear + other.linear,
            angular: self.angular + other.angular,
        }
    }
}

impl AddAssign for Impulse {
    fn add_assign(&mut self, other: Impulse) {
        *self = Impulse {
            linear: self.linear + other.linear,
            angular: self.angular + other.angular,
        };
    }
}

#[derive(Debug, Component)]
pub struct Body {
    pub enabled: bool,
    pub restitution: f32,
    pub velocity: Velocity,
    pub mass: BodyMass,
    pub impulse_accumulator: Impulse,
}

impl Default for Body {
    fn default() -> Body {
        Body {
            enabled: true,
            restitution: 0.,
            velocity: Velocity::default(),
            mass: BodyMass::default(),
            impulse_accumulator: Impulse::default(),
        }
    }
}

impl Body {
    pub fn add_impulse(&mut self, impulse: Impulse) {
        self.impulse_accumulator += impulse;
    }
}
