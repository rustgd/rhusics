pub use self::epa2d::EPA2D;
pub use self::epa3d::EPA3D;

mod epa2d;
mod epa3d;

use cgmath::prelude::*;
use collision::{Aabb, MinMax};

use super::SupportPoint;
use Real;
use collide::{CollisionPrimitive, Contact, Primitive};

pub const EPA_TOLERANCE: Real = 0.00001;
pub const MAX_ITERATIONS: u32 = 100;

pub trait EPA<T> {
    type Vector: VectorSpace<Scalar = Real> + ElementWise + Array<Element = Real>;
    type Point: EuclideanSpace<Scalar = Real, Diff = Self::Vector> + MinMax;
    type Aabb: Aabb<Scalar = Real, Diff = Self::Vector, Point = Self::Point>;
    type Primitive: Primitive<Self::Aabb>;

    fn process(
        &self,
        simplex: &mut Vec<SupportPoint<Self::Point>>,
        left: &CollisionPrimitive<Self::Primitive, Self::Aabb, T>,
        left_transform: &T,
        right: &CollisionPrimitive<Self::Primitive, Self::Aabb, T>,
        right_transform: &T,
    ) -> Vec<Contact<Self::Vector>>;

    fn new() -> Self;
}
