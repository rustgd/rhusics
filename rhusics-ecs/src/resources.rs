use std::fmt::Debug;

use cgmath::{BaseFloat, Basis2, EuclideanSpace, Point2, Point3, Quaternion, Rotation, Transform,
             Zero};
use collision::{Bound, Contains, Primitive, SurfaceArea, Union};
use collision::dbvt::{DynamicBoundingVolumeTree, TreeValue};
use shrev::EventChannel;
use specs::{Component, Entity, World};

use core::{Collider, CollisionShape, ContactEvent, ForceAccumulator, GetId, Mass, NextFrame, Pose,
           RigidBody, Velocity};
use physics::DeltaTime;

/// Utility method for registering collision types with `World`
pub trait WithRhusics {
    /// Register collision types
    ///
    /// ### Type parameters:
    ///
    /// - `P`: Collision Primitive
    /// - `B`: Bounding volume
    /// - `T`: Transform
    /// - `D`: TreeValue (used in DynamicBoundingVolumeTree)
    /// - `Y`: Collider
    fn register_collision<P, B, T, D, Y>(&mut self)
    where
        P: Primitive + Send + Sync + 'static,
        P::Point: Send + Sync + 'static,
        <P::Point as EuclideanSpace>::Diff: Debug + Send + Sync + 'static,
        <P::Point as EuclideanSpace>::Scalar: BaseFloat + Debug + Send + Sync + 'static,
        B: Bound<Point = P::Point>
            + Clone
            + Union<B, Output = B>
            + Contains<B>
            + SurfaceArea<Scalar = <P::Point as EuclideanSpace>::Scalar>
            + Send
            + Sync
            + 'static,
        T: Transform<P::Point> + Component + Send + Sync + 'static,
        D: TreeValue<Bound = B> + GetId<Entity> + Send + Sync + 'static,
        Y: Collider + Send + Sync + 'static;

    /// Register physics types
    ///
    /// Will also call `register_collision`
    ///
    /// ### Type parameters:
    ///
    /// - `P`: Collision Primitive
    /// - `B`: Bounding volume
    /// - `R`: Rotational quantity
    /// - `D`: TreeValue (used in DynamicBoundingVolumeTree)
    /// - `Y`: Collider
    /// - `L`: Linear velocity/force
    /// - `A`: Angular velocity/force
    /// - `I`: Inertia
    fn register_physics<P, B, R, D, Y, L, A, I, T>(&mut self)
    where
        P: Primitive + Send + Sync + 'static,
        P::Point: EuclideanSpace + Send + Sync + 'static,
        <P::Point as EuclideanSpace>::Diff: Debug + Send + Sync + 'static,
        <P::Point as EuclideanSpace>::Scalar: BaseFloat + Debug + Send + Sync + 'static,
        B: Bound<Point = P::Point>
            + Clone
            + Union<B, Output = B>
            + Contains<B>
            + SurfaceArea<Scalar = <P::Point as EuclideanSpace>::Scalar>
            + Send
            + Sync
            + 'static,
        R: Rotation<P::Point> + Send + Sync + 'static,
        T: Pose<P::Point, R> + Clone + Component + Send + Sync + 'static,
        D: TreeValue<Bound = B> + GetId<Entity> + Send + Sync + 'static,
        Y: Collider + Send + Sync + 'static,
        L: Clone + Send + Sync + 'static,
        A: Clone + Send + Sync + 'static,
        I: Send + Sync + 'static;

    /// Register physics types for 2D
    ///
    /// Will also call `register_collision`
    ///
    /// ### Type parameters:
    ///
    /// - `S`: Scalar (f32 or f64)
    /// - `P`: Collision Primitive
    /// - `B`: Bounding volume
    /// - `D`: TreeValue (used in DynamicBoundingVolumeTree)
    /// - `Y`: Collider
    fn register_physics_2d<S, P, B, D, Y, T>(&mut self)
    where
        P: Primitive<Point = Point2<S>> + Send + Sync + 'static,
        S: BaseFloat + Send + Sync + 'static,
        B: Bound<Point = P::Point>
            + Clone
            + Union<B, Output = B>
            + Contains<B>
            + SurfaceArea<Scalar = S>
            + Send
            + Sync
            + 'static,
        D: TreeValue<Bound = B> + GetId<Entity> + Send + Sync + 'static,
        T: Pose<Point2<S>, Basis2<S>> + Clone + Component + Send + Sync + 'static,
        Y: Collider + Send + Sync + 'static;

    /// Register physics types for 3D
    ///
    /// Will also call `register_collision`
    ///
    /// ### Type parameters:
    ///
    /// - `S`: Scalar (f32 or f64)
    /// - `P`: Collision Primitive
    /// - `B`: Bounding volume
    /// - `D`: TreeValue (used in DynamicBoundingVolumeTree)
    /// - `Y`: Collider
    fn register_physics_3d<S, P, B, D, Y, T>(&mut self)
    where
        P: Primitive<Point = Point3<S>> + Send + Sync + 'static,
        S: BaseFloat + Send + Sync + 'static,
        B: Bound<Point = P::Point>
            + Clone
            + Union<B, Output = B>
            + Contains<B>
            + SurfaceArea<Scalar = S>
            + Send
            + Sync
            + 'static,
        D: TreeValue<Bound = B> + GetId<Entity> + Send + Sync + 'static,
        T: Pose<Point3<S>, Quaternion<S>> + Clone + Component + Send + Sync + 'static,
        Y: Collider + Send + Sync + 'static;
}

impl WithRhusics for World {
    fn register_collision<P, B, T, D, Y>(&mut self)
    where
        P: Primitive + Send + Sync + 'static,
        P::Point: Send + Sync + 'static,
        <P::Point as EuclideanSpace>::Diff: Debug + Send + Sync + 'static,
        <P::Point as EuclideanSpace>::Scalar: BaseFloat + Debug + Send + Sync + 'static,
        B: Bound<Point = P::Point>
            + Clone
            + Union<B, Output = B>
            + Contains<B>
            + SurfaceArea<Scalar = <P::Point as EuclideanSpace>::Scalar>
            + Send
            + Sync
            + 'static,
        T: Transform<P::Point> + Component + Send + Sync + 'static,
        D: TreeValue<Bound = B> + GetId<Entity> + Send + Sync + 'static,
        Y: Collider + Send + Sync + 'static,
    {
        self.register::<T>();
        self.register::<NextFrame<T>>();
        self.register::<CollisionShape<P, T, B, Y>>();
        self.add_resource(EventChannel::<ContactEvent<Entity, P::Point>>::new());
        self.add_resource(DynamicBoundingVolumeTree::<D>::new());
    }

    fn register_physics<P, B, R, D, Y, L, A, I, T>(&mut self)
    where
        P: Primitive + Send + Sync + 'static,
        P::Point: EuclideanSpace + Send + Sync + 'static,
        <P::Point as EuclideanSpace>::Diff: Debug + Send + Sync + 'static,
        <P::Point as EuclideanSpace>::Scalar: BaseFloat + Debug + Send + Sync + 'static,
        B: Bound<Point = P::Point>
            + Clone
            + Union<B, Output = B>
            + Contains<B>
            + SurfaceArea<Scalar = <P::Point as EuclideanSpace>::Scalar>
            + Send
            + Sync
            + 'static,
        R: Rotation<P::Point> + Send + Sync + 'static,
        D: TreeValue<Bound = B> + GetId<Entity> + Send + Sync + 'static,
        Y: Collider + Send + Sync + 'static,
        L: Clone + Send + Sync + 'static,
        A: Clone + Send + Sync + 'static,
        I: Send + Sync + 'static,
        T: Pose<P::Point, R> + Clone + Component + Send + Sync + 'static,
    {
        self.add_resource(DeltaTime {
            delta_seconds: <P::Point as EuclideanSpace>::Scalar::zero(),
        });
        self.register::<Mass<<P::Point as EuclideanSpace>::Scalar, I>>();
        self.register::<Velocity<L, A>>();
        self.register::<NextFrame<Velocity<L, A>>>();
        self.register::<RigidBody<<P::Point as EuclideanSpace>::Scalar>>();
        self.register::<ForceAccumulator<L, A>>();
        self.register_collision::<P, B, T, D, Y>();
    }

    fn register_physics_2d<S, P, B, D, Y, T>(&mut self)
    where
        P: Primitive<Point = Point2<S>> + Send + Sync + 'static,
        S: BaseFloat + Send + Sync + 'static,
        B: Bound<Point = P::Point>
            + Clone
            + Union<B, Output = B>
            + Contains<B>
            + SurfaceArea<Scalar = S>
            + Send
            + Sync
            + 'static,
        D: TreeValue<Bound = B> + GetId<Entity> + Send + Sync + 'static,
        Y: Collider + Send + Sync + 'static,
        T: Pose<Point2<S>, Basis2<S>> + Clone + Component + Send + Sync + 'static,
    {
        use cgmath::Vector2;
        self.register_physics::<P, B, Basis2<S>, D, Y, Vector2<S>, S, S, T>()
    }

    fn register_physics_3d<S, P, B, D, Y, T>(&mut self)
    where
        P: Primitive<Point = Point3<S>> + Send + Sync + 'static,
        S: BaseFloat + Send + Sync + 'static,
        B: Bound<Point = P::Point>
            + Clone
            + Union<B, Output = B>
            + Contains<B>
            + SurfaceArea<Scalar = S>
            + Send
            + Sync
            + 'static,
        D: TreeValue<Bound = B> + GetId<Entity> + Send + Sync + 'static,
        Y: Collider + Send + Sync + 'static,
        T: Pose<Point3<S>, Quaternion<S>> + Clone + Component + Send + Sync + 'static,
    {
        use cgmath::{Matrix3, Vector3};
        self.register_physics::<P, B, Quaternion<S>, D, Y, Vector3<S>, Vector3<S>, Matrix3<S>, T>()
    }
}
