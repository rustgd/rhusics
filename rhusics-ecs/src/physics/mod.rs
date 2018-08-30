//! Contains physics components, resources and systems for use with `specs`
pub use self::resources::*;
pub use self::setup::*;
pub use self::systems::*;

mod resources;
mod setup;
mod systems;
