//! Contains physics components, resources and systems for use with `specs`
pub use self::resources::*;
pub use self::systems::*;

pub mod prelude2d;
pub mod prelude3d;

mod resources;
mod systems;
