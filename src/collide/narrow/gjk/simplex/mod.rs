pub use self::simplex2d::SimplexProcessor2D;
pub use self::simplex3d::SimplexProcessor3D;

mod simplex2d;
mod simplex3d;

use cgmath::num_traits::Float;
use cgmath::prelude::*;

use Real;

pub trait SimplexProcessor {
    type Vector: VectorSpace<Scalar = Real>;

    fn check_origin(&self, simplex: &mut Vec<Self::Vector>, d: &mut Self::Vector) -> bool;
    fn closest_feature(&self, simplex: &Vec<Self::Vector>) -> Option<Feature<Self::Vector>>;
    fn new() -> Self;
}

#[derive(Debug)]
pub struct Feature<V>
where
    V: VectorSpace,
{
    pub normal: V,
    pub distance: V::Scalar,
    pub index: usize,
}

impl<V> Feature<V>
where
    V: VectorSpace<Scalar = Real>,
{
    pub fn new() -> Self {
        Self {
            normal: V::zero(),
            distance: Real::infinity(),
            index: 0,
        }
    }
}