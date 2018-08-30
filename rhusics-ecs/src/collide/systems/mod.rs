//! Contains systems for collision detection and spatial querying

pub use self::basic::BasicCollisionSystem;
pub use self::spatial_collision::SpatialCollisionSystem;
pub use self::spatial_sort::SpatialSortingSystem;

mod basic;
mod spatial_collision;
mod spatial_sort;
