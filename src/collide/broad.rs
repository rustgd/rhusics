//! Broad phase

pub use collision::algorithm::broad_phase::*;

use collision::prelude::*;

/// Broad phase
pub trait BroadPhase<A>
where
    A: HasBound,
{
    /// Compute potential collider pairs
    fn find_potentials(&mut self, shapes: &mut [A]) -> Vec<(usize, usize)>;
}

impl<A> BroadPhase<A> for BruteForce
where
    A: HasBound,
    A::Bound: Aabb + Discrete<A::Bound>,
{
    fn find_potentials(&mut self, shapes: &mut [A]) -> Vec<(usize, usize)> {
        self.find_collider_pairs(shapes)
    }
}

impl<A, V> BroadPhase<A> for SweepAndPrune<V>
where
    A: HasBound,
    A::Bound: Aabb + Discrete<A::Bound>,
    V: Variance<Bound = A::Bound>,
{
    fn find_potentials(&mut self, shapes: &mut [A]) -> Vec<(usize, usize)> {
        self.find_collider_pairs(shapes)
    }
}
