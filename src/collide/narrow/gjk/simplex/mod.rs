pub use self::simplex2d::SimplexProcessor2D;
pub use self::simplex3d::SimplexProcessor3D;

mod simplex2d;
mod simplex3d;

use cgmath::prelude::*;

use super::SupportPoint;
use Real;

pub trait SimplexProcessor {
    type Vector: VectorSpace<Scalar = Real>;
    type Point: EuclideanSpace<Scalar = Real, Diff = Self::Vector>;

    fn check_origin(&self, simplex: &mut Vec<SupportPoint<Self::Point>>, d: &mut Self::Vector) -> bool;
    fn new() -> Self;
}
