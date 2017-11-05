use specs::{Component, DenseVecStorage};

use Real;
use physics::{ForceAccumulator, Mass, RigidBody, Velocity};

impl<V> Component for Velocity<V>
where
    V: Send + Sync + 'static + Clone,
{
    type Storage = DenseVecStorage<Self>;
}

impl<I> Component for Mass<I> {
    type Storage = DenseVecStorage<Self>;
}

impl Component for RigidBody {
    type Storage = DenseVecStorage<Self>;
}

impl<V> Component for ForceAccumulator<V>
where
    V: Send + Sync + 'static,
{
    type Storage = DenseVecStorage<Self>;
}

/// Used for computations
pub struct DeltaTime {
    /// Delta time since last frame
    pub delta_seconds: Real,
}
