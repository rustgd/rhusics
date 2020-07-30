//! Broad phase

pub use collision::algorithm::broad_phase::*;

use collision::prelude::*;

use super::{CollisionData, GetId};

/// Broad phase
///
/// ### Type parameters:
///
/// - `A`: Shape type, must be able to return its bounding volume
pub trait BroadPhase<A>: Send
where
    A: HasBound,
{
    /// Compute potential collider pairs
    fn find_potentials(&mut self, shapes: &mut [A]) -> Vec<(usize, usize)>;
}

impl<A> BroadPhase<A> for BruteForce
where
    A: HasBound,
    A::Bound: Discrete<A::Bound>,
{
    fn find_potentials(&mut self, shapes: &mut [A]) -> Vec<(usize, usize)> {
        self.find_collider_pairs(shapes)
    }
}

impl<A, V> BroadPhase<A> for SweepAndPrune<V>
where
    A: HasBound,
    A::Bound: Discrete<A::Bound>,
    V: Variance<Bound = A::Bound> + Send,
{
    fn find_potentials(&mut self, shapes: &mut [A]) -> Vec<(usize, usize)> {
        self.find_collider_pairs(shapes)
    }
}

/// Perform broad phase collision detection on the given data, with the given broad phase
/// Will return a list of ids provided by `GetId` from the broad phase data from `C`
///
/// ### Type parameters:
///
/// - `C`: Collision data
/// - `I`: Id, returned by `GetId` on `D`, primary id for a collider
/// - `P`: Primitive
/// - `T`: Transform
/// - `B`: Bounding volume
/// - `Y`: Collider, see `Collider` for more information
/// - `D`: Broad phase data
pub fn broad_collide<C, I, P, T, B, Y, D>(data: &C, broad: &mut Box<dyn BroadPhase<D>>) -> Vec<(I, I)>
where
    C: CollisionData<I, P, T, B, Y, D>,
    P: Primitive,
    D: HasBound<Bound = B> + GetId<I>,
    B: Bound<Point = P::Point>,
{
    let mut info = data.get_broad_data();
    broad
        .find_potentials(&mut info)
        .iter()
        .map(|&(a, b)| (info[a].id(), info[b].id()))
        .collect::<Vec<_>>()
}
