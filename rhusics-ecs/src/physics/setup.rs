use std::fmt::Debug;
use std::ops::{Add, Mul, Sub};

use cgmath::{BaseFloat, Basis2, EuclideanSpace, InnerSpace, Matrix3, Point2, Point3, Quaternion,
             Rotation, Transform, Vector3, Zero};
use collision::{Bound, ComputeBound, Contains, Discrete, HasBound, SurfaceArea, Union};
use collision::dbvt::TreeValue;
use core::{ApplyAngular, BroadPhase, GetId, Inertia, NarrowPhase, PartialCrossProduct,
           PhysicsTime, Pose, Primitive};
use specs::prelude::{Component, DispatcherBuilder, Entity, Tracked};

/// Create systems and add to a `Dispatcher` graph.
///
/// ### Parameters
///
/// - `dispatcher`: The dispatcher to add the systems to.
/// - `broad_phase`: Broad phase to use
/// - `narrow_phase`: Narrow phase to use
/// - `spatial`: If spatial or basic collision detection should be used
///
/// ### Type parameters:
///
/// - `P`: Shape primitive
/// - `T`: Pose (transform)
/// - `B`: Bounding volume
/// - `D`: Broad phase data, usually `TreeValueWrapped`.
/// - `Y`: Collider
/// - `V`: Broad phase algorithm
/// - `N`: Narrow phase algorithm
/// - `R`: Rotational quantity, `Basis2` or `Quaternion`
/// - `A`: Angular velocity, `Scalar` or `Vector3`
/// - `I`: Inertia, `Scalar` or `Matrix3`
/// - `DT`: Time quantity, usually `DeltaTime`
/// - `O`: Internal type used to abstract cross product for 2D vs 3D, `Scalar` or `Vector3`
pub fn setup_dispatch<'a, 'b, P, T, B, D, Y, V, N, R, A, I, DT, O>(
    dispatcher: &mut DispatcherBuilder<'a, 'b>,
    broad_phase: V,
    narrow_phase: N,
    spatial: bool,
) where
    V: BroadPhase<D> + BroadPhase<(usize, D)> + 'static,
    N: NarrowPhase<P, T, B, Y> + 'static,
    P: Primitive + ComputeBound<B> + Send + Sync + 'static,
    P::Point: Debug + Send + Sync + 'static,
    <P::Point as EuclideanSpace>::Scalar: BaseFloat + Send + Sync + 'static,
    <P::Point as EuclideanSpace>::Diff: InnerSpace
        + PartialCrossProduct<<P::Point as EuclideanSpace>::Diff, Output = O>
        + Debug
        + Send
        + Sync
        + 'static,
    T: Debug + Component + Pose<P::Point, R> + Transform<P::Point> + Send + Sync + Clone + 'static,
    T::Storage: Tracked,
    Y: Default + Send + Sync + 'static,
    B: Bound<Point = P::Point>
        + Send
        + Sync
        + 'static
        + Union<B, Output = B>
        + Clone
        + Contains<B>
        + SurfaceArea<Scalar = <P::Point as EuclideanSpace>::Scalar>
        + Discrete<B>
        + Debug,
    (usize, D): HasBound<Bound = B>,
    D: TreeValue<Bound = B>
        + HasBound<Bound = B>
        + From<(Entity, B)>
        + GetId<Entity>
        + Send
        + Sync
        + 'static,
    R: Rotation<P::Point>
        + ApplyAngular<<P::Point as EuclideanSpace>::Scalar, A>
        + Send
        + Sync
        + 'static,
    I: Inertia<Orientation = R> + Mul<A, Output = A> + Mul<O, Output = O> + Send + Sync + 'static,
    A: Mul<<P::Point as EuclideanSpace>::Scalar, Output = A>
        + PartialCrossProduct<
        <P::Point as EuclideanSpace>::Diff,
        Output = <P::Point as EuclideanSpace>::Diff,
    >
        + Zero
        + Clone
        + Copy
        + Send
        + Sync
        + 'static,
    DT: PhysicsTime<<P::Point as EuclideanSpace>::Scalar> + Default + Send + Sync + 'static,
    O: PartialCrossProduct<
        <P::Point as EuclideanSpace>::Diff,
        Output = <P::Point as EuclideanSpace>::Diff,
    >
        + Send
        + Sync
        + 'static,
    for<'c> &'c A: Sub<O, Output = A> + Add<O, Output = A>,
{
    use {BasicCollisionSystem, ContactResolutionSystem, CurrentFrameUpdateSystem,
         NextFrameSetupSystem, SpatialCollisionSystem, SpatialSortingSystem};
    dispatcher.add(
        CurrentFrameUpdateSystem::<P::Point, R, A, T>::new(),
        "physics_solver_system",
        &[],
    );
    dispatcher.add(
        NextFrameSetupSystem::<P::Point, R, I, A, T, DT>::new(),
        "next_frame_setup",
        &["physics_solver_system"],
    );
    if spatial {
        dispatcher.add(
            SpatialSortingSystem::<P, T, D, B, Y>::new(),
            "spatial_sorting_system",
            &["next_frame_setup"],
        );
        dispatcher.add(
            SpatialCollisionSystem::<P, T, (usize, D), B, Y>::new()
                .with_broad_phase(broad_phase)
                .with_narrow_phase(narrow_phase),
            "collision_system",
            &["spatial_sorting_system"],
        );
    } else {
        dispatcher.add(
            BasicCollisionSystem::<P, T, D, B, Y>::new()
                .with_broad_phase(broad_phase)
                .with_narrow_phase(narrow_phase),
            "collision_system",
            &["next_frame_setup"],
        );
    }
    dispatcher.add(
        ContactResolutionSystem::<P::Point, R, I, A, O, T>::new(),
        "contact_resolution",
        &["collision_system"],
    );
}

/// Create systems for 2D and add to a `Dispatcher` graph.
///
/// ### Parameters
///
/// - `dispatcher`: The dispatcher to add the systems to.
/// - `broad_phase`: Broad phase to use
/// - `narrow_phase`: Narrow phase to use
/// - `spatial`: If spatial or basic collision detection should be used
///
/// ### Type parameters:
///
/// - `S`: Scalar type, `f32` or `f64`
/// - `P`: Shape primitive
/// - `T`: Pose (transform)
/// - `B`: Bounding volume
/// - `D`: Broad phase data, usually `TreeValueWrapped`.
/// - `Y`: Collider
/// - `V`: Broad phase algorithm
/// - `N`: Narrow phase algorithm
/// - `DT`: Time quantity, usually `DeltaTime`
pub fn setup_dispatch_2d<'a, 'b, S, P, T, B, D, Y, V, N, DT>(
    dispatcher: &mut DispatcherBuilder<'a, 'b>,
    broad_phase: V,
    narrow_phase: N,
    spatial: bool,
) where
    V: BroadPhase<D> + BroadPhase<(usize, D)> + 'static,
    N: NarrowPhase<P, T, B, Y> + 'static,
    P: Primitive<Point = Point2<S>> + ComputeBound<B> + Send + Sync + 'static,
    S: Inertia<Orientation = Basis2<S>> + BaseFloat + Send + Sync + 'static,
    T: Component
        + Pose<Point2<S>, Basis2<S>>
        + Debug
        + Transform<Point2<S>>
        + Send
        + Sync
        + Clone
        + 'static,
    T::Storage: Tracked,
    Y: Default + Send + Sync + 'static,
    B: Bound<Point = Point2<S>>
        + Send
        + Sync
        + 'static
        + Union<B, Output = B>
        + Clone
        + Contains<B>
        + SurfaceArea<Scalar = S>
        + Discrete<B>
        + Debug,
    (usize, D): HasBound<Bound = B>,
    D: TreeValue<Bound = B>
        + HasBound<Bound = B>
        + From<(Entity, B)>
        + GetId<Entity>
        + Send
        + Sync
        + 'static,
    DT: PhysicsTime<S> + Default + Send + Sync + 'static,
    for<'c> &'c S: Sub<S, Output = S> + Add<S, Output = S>,
{
    setup_dispatch::<P, T, B, D, Y, V, N, Basis2<S>, S, S, DT, S>(
        dispatcher,
        broad_phase,
        narrow_phase,
        spatial,
    );
}

/// Create systems for 3sD and add to a `Dispatcher` graph.
///
/// ### Parameters
///
/// - `dispatcher`: The dispatcher to add the systems to.
/// - `broad_phase`: Broad phase to use
/// - `narrow_phase`: Narrow phase to use
/// - `spatial`: If spatial or basic collision detection should be used
///
/// ### Type parameters:
///
/// - `S`: Scalar type, `f32` or `f64`
/// - `P`: Shape primitive
/// - `T`: Pose (transform)
/// - `B`: Bounding volume
/// - `D`: Broad phase data, usually `TreeValueWrapped`.
/// - `Y`: Collider
/// - `V`: Broad phase algorithm
/// - `N`: Narrow phase algorithm
/// - `DT`: Time quantity, usually `DeltaTime`
pub fn setup_dispatch_3d<'a, 'b, S, P, T, B, D, Y, V, N, DT>(
    dispatcher: &mut DispatcherBuilder<'a, 'b>,
    broad_phase: V,
    narrow_phase: N,
    spatial: bool,
) where
    V: BroadPhase<D> + BroadPhase<(usize, D)> + 'static,
    N: NarrowPhase<P, T, B, Y> + 'static,
    P: Primitive<Point = Point3<S>> + ComputeBound<B> + Send + Sync + 'static,
    S: BaseFloat + Send + Sync + 'static,
    T: Component
        + Pose<Point3<S>, Quaternion<S>>
        + Transform<Point3<S>>
        + Debug
        + Send
        + Sync
        + Clone
        + 'static,
    T::Storage: Tracked,
    Y: Default + Send + Sync + 'static,
    B: Bound<Point = Point3<S>>
        + Send
        + Sync
        + 'static
        + Union<B, Output = B>
        + Clone
        + Contains<B>
        + SurfaceArea<Scalar = S>
        + Discrete<B>
        + Debug,
    (usize, D): HasBound<Bound = B>,
    D: TreeValue<Bound = B>
        + HasBound<Bound = B>
        + From<(Entity, B)>
        + GetId<Entity>
        + Send
        + Sync
        + 'static,
    DT: PhysicsTime<S> + Default + Send + Sync + 'static,
{
    setup_dispatch::<P, T, B, D, Y, V, N, Quaternion<S>, Vector3<S>, Matrix3<S>, DT, Vector3<S>>(
        dispatcher,
        broad_phase,
        narrow_phase,
        spatial,
    );
}

#[cfg(test)]
mod tests {

    use super::*;
    use DeltaTime;
    use collide2d::{BodyPose2, GJK2, SweepAndPrune2};
    use collide3d::{BodyPose3, GJK3, SweepAndPrune3};
    use collision::{Aabb2, Aabb3};
    use collision::dbvt::TreeValueWrapped;
    use collision::primitive::{Primitive2, Primitive3};

    #[test]
    fn test_dispatch() {
        let mut builder = DispatcherBuilder::new();
        setup_dispatch::<
            Primitive2<f32>,
            BodyPose2<f32>,
            Aabb2<f32>,
            TreeValueWrapped<Entity, Aabb2<f32>>,
            (),
            _,
            _,
            _,
            _,
            f32,
            DeltaTime<f32>,
            _,
        >(&mut builder, SweepAndPrune2::new(), GJK2::new(), false);
    }

    #[test]
    fn test_dispatch_2d() {
        let mut builder = DispatcherBuilder::new();
        setup_dispatch_2d::<
            _,
            Primitive2<f32>,
            BodyPose2<f32>,
            Aabb2<f32>,
            TreeValueWrapped<Entity, Aabb2<f32>>,
            (),
            _,
            _,
            DeltaTime<f32>,
        >(&mut builder, SweepAndPrune2::new(), GJK2::new(), false);
    }

    #[test]
    fn test_dispatch_3d() {
        let mut builder = DispatcherBuilder::new();
        setup_dispatch_3d::<
            _,
            Primitive3<f32>,
            BodyPose3<f32>,
            Aabb3<f32>,
            TreeValueWrapped<Entity, Aabb3<f32>>,
            (),
            _,
            _,
            DeltaTime<f32>,
        >(&mut builder, SweepAndPrune3::new(), GJK3::new(), false);
    }
}
