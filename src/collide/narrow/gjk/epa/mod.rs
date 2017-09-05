pub use self::epa2d::EPA2D;
pub use self::epa3d::EPA3D;

mod epa2d;
mod epa3d;

use cgmath::prelude::*;
use collision::MinMax;

use super::SupportPoint;
use Real;
use collide::{CollisionPrimitive, Contact, Primitive};

pub const EPA_TOLERANCE: Real = 0.00001;
pub const MAX_ITERATIONS: u32 = 100;

pub trait EPA<T> {
    type Vector: VectorSpace<Scalar = Real> + ElementWise + Array<Element = Real>;
    type Point: EuclideanSpace<Scalar = Real, Diff = Self::Vector> + MinMax;
    type Primitive: Primitive<Vector = Self::Vector, Point = Self::Point>;

    fn process(
        &self,
        simplex: &mut Vec<SupportPoint<Self::Point>>,
        left: &CollisionPrimitive<Self::Primitive, T>,
        left_transform: &T,
        right: &CollisionPrimitive<Self::Primitive, T>,
        right_transform: &T,
    ) -> Vec<Contact<Self::Vector>>;

    fn new() -> Self;
}
