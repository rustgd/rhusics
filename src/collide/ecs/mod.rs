//! Contains resources and systems usable for collision detection in
//! [`specs`](https://docs.rs/specs/0.9.5/specs/)
//!

pub use self::resources::Contacts;
pub use self::system::CollisionSystem;

mod resources;
mod system;
