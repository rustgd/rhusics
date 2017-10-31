use specs::{Component, DenseVecStorage};

use Real;
use physics::{Mass, Velocity};

impl<V> Component for Velocity<V>
where
    V: Send + Sync + 'static + Clone,
{
    type Storage = DenseVecStorage<Self>;
}

impl Component for Mass {
    type Storage = DenseVecStorage<Self>;
}

/// Used for computations
pub struct DeltaTime {
    /// Delta time since last frame
    pub delta_seconds: Real,
}
