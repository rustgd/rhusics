pub use self::epa2d::EPA2;
pub use self::epa3d::EPA3;

mod epa2d;
mod epa3d;

use cgmath::prelude::*;
use collision::prelude::*;

use super::SupportPoint;
use Real;
use collide::Contact;

pub const EPA_TOLERANCE: Real = 0.00001;
pub const MAX_ITERATIONS: u32 = 100;

pub trait EPA {
    type Point: EuclideanSpace<Scalar = Real>;

    fn process<SL, SR, TL, TR>(
        &self,
        simplex: &mut Vec<SupportPoint<Self::Point>>,
        left: &SL,
        left_transform: &TL,
        right: &SR,
        right_transform: &TR,
    ) -> Option<Contact<Self::Point>>
    where
        SL: SupportFunction<Point = Self::Point>,
        SR: SupportFunction<Point = Self::Point>,
        TL: Transform<Self::Point>,
        TR: Transform<Self::Point>;


    fn new() -> Self;
}
