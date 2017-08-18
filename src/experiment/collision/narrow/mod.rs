use experiment::collision::{CollisionShape, CollisionEvent};
use experiment::BodyPose;

pub mod gjk;

pub trait NarrowPhase {
    fn collide(
        &mut self,
        left: &mut (CollisionShape, BodyPose),
        right: &mut (CollisionShape, BodyPose),
    ) -> Option<CollisionEvent>;
}
