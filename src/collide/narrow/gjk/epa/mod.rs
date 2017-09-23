pub use self::epa2d::EPA2;
pub use self::epa3d::EPA3;

mod epa2d;
mod epa3d;

use super::SupportPoint;
use Real;
use collide::Contact;
use collide::primitives::SupportFunction;

pub const EPA_TOLERANCE: Real = 0.00001;
pub const MAX_ITERATIONS: u32 = 100;

pub trait EPA<P, T>
where
    P : SupportFunction,
{
    fn process(
        &self,
        simplex: &mut Vec<SupportPoint<P::Point>>,
        left: &P,
        left_transform: &T,
        right: &P,
        right_transform: &T,
    ) -> Vec<Contact<P::Point>>;

    fn new() -> Self;
}
