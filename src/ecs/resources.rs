use std::fmt::Debug;

use cgmath::{EuclideanSpace, Rotation, Transform};
use collision::{Bound, Contains, Primitive, SurfaceArea, Union};
use collision::dbvt::{DynamicBoundingVolumeTree, TreeValue};
use shrev::EventChannel;
use specs::{Component, Entity, World};

use {BodyPose, NextFrame, Real};
use collide::{Collider, CollisionShape, ContactEvent};
use ecs::collide::GetEntity;
use ecs::physics::DeltaTime;
use physics::{ForceAccumulator, Mass, RigidBody, Velocity};

/// Utility method for registering collision types with `World`
pub trait WithRhusics {
    /// Register collision types
    fn register_collision<P, B, T, D, Y>(&mut self)
    where
        P: Primitive + Send + Sync + 'static,
        P::Point: Send + Sync + 'static,
        <P::Point as EuclideanSpace>::Diff: Debug + Send + Sync + 'static,
        <P::Point as EuclideanSpace>::Scalar: Debug + Send + Sync + 'static,
        B: Bound<Point = P::Point>
            + Clone
            + Union<B, Output = B>
            + Contains<B>
            + SurfaceArea<Scalar = <P::Point as EuclideanSpace>::Scalar>
            + Send
            + Sync
            + 'static,
        T: Transform<P::Point> + Component + Send + Sync + 'static,
        D: TreeValue<Bound = B> + GetEntity + Send + Sync + 'static,
        Y: Collider + Send + Sync + 'static;

    /// Register physics types
    ///
    /// Will also call `register_collision`
    fn register_physics<P, B, R, D, Y, L, A, I>(&mut self)
    where
        P: Primitive + Send + Sync + 'static,
        P::Point: EuclideanSpace<Scalar = Real> + Send + Sync + 'static,
        <P::Point as EuclideanSpace>::Diff: Debug + Send + Sync + 'static,
        B: Bound<Point = P::Point>
            + Clone
            + Union<B, Output = B>
            + Contains<B>
            + SurfaceArea<Scalar = <P::Point as EuclideanSpace>::Scalar>
            + Send
            + Sync
            + 'static,
        R: Rotation<P::Point> + Send + Sync + 'static,
        D: TreeValue<Bound = B> + GetEntity + Send + Sync + 'static,
        Y: Collider + Send + Sync + 'static,
        L: Clone + Send + Sync + 'static,
        A: Clone + Send + Sync + 'static,
        I: Send + Sync + 'static;
}

impl WithRhusics for World {
    fn register_collision<P, B, T, D, Y>(&mut self)
    where
        P: Primitive + Send + Sync + 'static,
        P::Point: Send + Sync + 'static,
        <P::Point as EuclideanSpace>::Diff: Debug + Send + Sync + 'static,
        <P::Point as EuclideanSpace>::Scalar: Debug + Send + Sync + 'static,
        B: Bound<Point = P::Point>
            + Clone
            + Union<B, Output = B>
            + Contains<B>
            + SurfaceArea<Scalar = <P::Point as EuclideanSpace>::Scalar>
            + Send
            + Sync
            + 'static,
        T: Transform<P::Point> + Component + Send + Sync + 'static,
        D: TreeValue<Bound = B> + GetEntity + Send + Sync + 'static,
        Y: Collider + Send + Sync + 'static,
    {
        self.register::<T>();
        self.register::<NextFrame<T>>();
        self.register::<CollisionShape<P, T, B, Y>>();
        self.add_resource(EventChannel::<ContactEvent<Entity, P::Point>>::new());
        self.add_resource(DynamicBoundingVolumeTree::<D>::new());
    }

    fn register_physics<P, B, R, D, Y, L, A, I>(&mut self)
    where
        P: Primitive + Send + Sync + 'static,
        P::Point: EuclideanSpace<Scalar = Real> + Send + Sync + 'static,
        <P::Point as EuclideanSpace>::Diff: Debug + Send + Sync + 'static,
        B: Bound<Point = P::Point>
            + Clone
            + Union<B, Output = B>
            + Contains<B>
            + SurfaceArea<Scalar = <P::Point as EuclideanSpace>::Scalar>
            + Send
            + Sync
            + 'static,
        R: Rotation<P::Point> + Send + Sync + 'static,
        D: TreeValue<Bound = B> + GetEntity + Send + Sync + 'static,
        Y: Collider + Send + Sync + 'static,
        L: Clone + Send + Sync + 'static,
        A: Clone + Send + Sync + 'static,
        I: Send + Sync + 'static,
    {
        self.add_resource(DeltaTime { delta_seconds: 0. });
        self.register::<Mass<I>>();
        self.register::<Velocity<L, A>>();
        self.register::<NextFrame<Velocity<L, A>>>();
        self.register::<RigidBody>();
        self.register::<ForceAccumulator<L, A>>();
        self.register_collision::<P, B, BodyPose<P::Point, R>, D, Y>();
    }
}
