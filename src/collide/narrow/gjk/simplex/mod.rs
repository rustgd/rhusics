pub use self::simplex2d::SimplexProcessor2;
pub use self::simplex3d::SimplexProcessor3;

mod simplex2d;
mod simplex3d;

use cgmath::prelude::*;

use super::SupportPoint;
use Real;

pub trait SimplexProcessor {
    type Point: EuclideanSpace<Scalar = Real>;

    fn check_origin(
        &self,
        simplex: &mut Vec<SupportPoint<Self::Point>>,
        d: &mut <Self::Point as EuclideanSpace>::Diff,
    ) -> bool;
    fn new() -> Self;
}
