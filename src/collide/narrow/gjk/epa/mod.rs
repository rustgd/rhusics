pub use self::epa2d::EPA2;
pub use self::epa3d::EPA3;

mod epa2d;
mod epa3d;

use cgmath::prelude::*;
use collision::prelude::*;

use super::SupportPoint;
use {Pose, Real};
use collide::Contact;

pub const EPA_TOLERANCE: Real = 0.00001;
pub const MAX_ITERATIONS: u32 = 100;

pub trait EPA {
    type Point: EuclideanSpace<Scalar = Real> + MinMax;

    fn process<P, T>(
        &self,
        simplex: &mut Vec<SupportPoint<P::Point>>,
        left: &P,
        left_transform: &T,
        right: &P,
        right_transform: &T,
    ) -> Vec<Contact<P::Point>>
    where
        P: SupportFunction<Point = Self::Point>,
        T: Pose<Self::Point>;

    fn new() -> Self;
}
