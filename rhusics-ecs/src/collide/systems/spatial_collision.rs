use std::fmt::Debug;

use cgmath::prelude::*;
use cgmath::BaseFloat;
use collision::dbvt::{DynamicBoundingVolumeTree, TreeValue};
use collision::prelude::*;
use shrev::EventChannel;
use specs::prelude::{
    BitSet, Component, Entities, Entity, Join, ReadStorage, ReaderId, ComponentEvent,
    Resources, System, Tracked, Write,
};

use core::{
    tree_collide, BroadPhase, CollisionData, CollisionShape, ContactEvent, GetId, NarrowPhase,
    NextFrame, Primitive,
};

/// Collision detection [system](https://docs.rs/specs/0.9.5/specs/trait.System.html) for use with
/// [`specs`](https://docs.rs/specs/0.9.5/specs/).
///
/// Will perform spatial sorting of the collision world.
///
/// Has support for both broad phase and narrow phase collision detection. Will only do narrow phase
/// if both broad and narrow phase is activated. If no broad phase is set, it will use a DBVT based
/// broad phase that has complexity O(m log^2 n), where m is the number of shapes that have a dirty
/// pose.
///
/// Can handle any transform component type, as long as the type implements
/// [`Transform`](https://docs.rs/cgmath/0.15.0/cgmath/trait.Transform.html), and as long as the
/// storage is wrapped in
/// [`FlaggedStorage`](https://docs.rs/specs/0.9.5/specs/struct.FlaggedStorage.html).
///
/// ### Type parameters:
///
/// - `P`: Shape primitive
/// - `T`: Transform
/// - `D`: Data accepted by broad phase
/// - `Y`: Shape type, see `Collider`
///
/// ### System Function:
///
/// `fn(Entities, T, NextFrame<T>, CollisionShape, DynamicBoundingVolumeTree<D>) -> (DynamicBoundingVolumeTree<D>, EventChannel<ContactEvent>)`
pub struct SpatialCollisionSystem<P, T, D, B, Y = ()>
where
    P: Primitive,
    B: Bound,
{
    narrow: Option<Box<NarrowPhase<P, T, B, Y>>>,
    broad: Option<Box<BroadPhase<D>>>,
    dirty: BitSet,
    pose_reader: Option<ReaderId<ComponentEvent>>,
    next_pose_reader: Option<ReaderId<ComponentEvent>>,

}

impl<P, T, D, B, Y> SpatialCollisionSystem<P, T, D, B, Y>
where
    P: Primitive + Send + Sync + 'static,
    <P::Point as EuclideanSpace>::Diff: Debug,
    <P::Point as EuclideanSpace>::Scalar: BaseFloat,
    B: Clone
        + Debug
        + Send
        + Sync
        + 'static
        + Bound<Point = P::Point>
        + Union<B, Output = B>
        + Contains<B>
        + SurfaceArea<Scalar = <P::Point as EuclideanSpace>::Scalar>,
    T: Transform<P::Point> + Component,
    D: HasBound<Bound = B>,
{
    /// Create a new collision detection system, with no broad or narrow phase activated.
    pub fn new() -> Self {
        SpatialCollisionSystem {
            narrow: None,
            broad: None,
            dirty: BitSet::default(),
            pose_reader: None,
            next_pose_reader: None,
        }
    }

    /// Specify what narrow phase algorithm to use
    pub fn with_narrow_phase<N: NarrowPhase<P, T, B, Y> + 'static>(mut self, narrow: N) -> Self {
        self.narrow = Some(Box::new(narrow));
        self
    }

    /// Specify what broad phase algorithm to use
    pub fn with_broad_phase<V: BroadPhase<D> + 'static>(mut self, broad: V) -> Self {
        self.broad = Some(Box::new(broad));
        self
    }
}

impl<'a, P, T, Y, B, D> System<'a> for SpatialCollisionSystem<P, T, (usize, D), B, Y>
where
    P: Primitive + ComputeBound<B> + Send + Sync + 'static,
    P::Point: EuclideanSpace,
    <P::Point as EuclideanSpace>::Scalar: BaseFloat + Send + Sync + 'static,
    B: Clone
        + Debug
        + Send
        + Sync
        + 'static
        + Bound<Point = P::Point>
        + Union<B, Output = B>
        + Discrete<B>
        + Contains<B>
        + SurfaceArea<Scalar = <P::Point as EuclideanSpace>::Scalar>,
    <P::Point as EuclideanSpace>::Diff: Debug + Send + Sync + 'static,
    P::Point: Debug + Send + Sync + 'static,
    T: Component + Clone + Debug + Transform<P::Point> + Send + Sync + 'static,
    T::Storage: Tracked,
    Y: Default + Send + Sync + 'static,
    D: Send + Sync + 'static + TreeValue<Bound = B> + HasBound<Bound = B> + GetId<Entity>,
{
    type SystemData = (
        Entities<'a>,
        ReadStorage<'a, T>,
        ReadStorage<'a, NextFrame<T>>,
        ReadStorage<'a, CollisionShape<P, T, B, Y>>,
        Write<'a, EventChannel<ContactEvent<Entity, P::Point>>>,
        Write<'a, DynamicBoundingVolumeTree<D>>,
    );

    fn run(&mut self, system_data: Self::SystemData) {
        let (entities, poses, next_poses, shapes, mut event_channel, mut tree) = system_data;
        self.dirty.clear();

        for event in poses.channel().read(self.pose_reader.as_mut().unwrap()) {
            match event {
                ComponentEvent::Inserted(index) => { self.dirty.add(*index); },
                ComponentEvent::Modified(index) => { self.dirty.add(*index); },
                ComponentEvent::Removed(index) => {
                    self.dirty.remove(*index);
                }
            }
        }
        for event in next_poses.channel().read(self.next_pose_reader.as_mut().unwrap()) {
            match event {
                ComponentEvent::Inserted(index) => { self.dirty.add(*index); },
                ComponentEvent::Modified(index) => { self.dirty.add(*index); },
                ComponentEvent::Removed(index) => {
                    self.dirty.remove(*index);
                }
            }
        }

        event_channel.iter_write(tree_collide(
            &SpatialCollisionData {
                poses,
                shapes,
                next_poses,
                entities,
                dirty: &self.dirty,
            },
            &mut *tree,
            &mut self.broad,
            &self.narrow,
        ));
    }

    fn setup(&mut self, res: &mut Resources) {
        use specs::prelude::{SystemData, WriteStorage};
        Self::SystemData::setup(res);
        let mut poses = WriteStorage::<T>::fetch(res);
        self.pose_reader = Some(poses.register_reader());
        let mut next_poses = WriteStorage::<NextFrame<T>>::fetch(res);
        self.next_pose_reader = Some(next_poses.register_reader());
    }
}

/// Collision data used by ECS systems
pub struct SpatialCollisionData<'a, P, T, B, Y>
where
    P: Primitive + ComputeBound<B> + Send + Sync + 'static,
    P::Point: Debug + Send + Sync + 'static,
    <P::Point as EuclideanSpace>::Scalar: Send + Sync + 'static,
    <P::Point as EuclideanSpace>::Diff: Debug + Send + Sync + 'static,
    T: Component + Transform<P::Point> + Send + Sync + Clone + 'static,
    Y: Default + Send + Sync + 'static,
    B: Bound<Point = P::Point> + Send + Sync + 'static + Union<B, Output = B> + Clone,
{
    /// collision shapes
    pub shapes: ReadStorage<'a, CollisionShape<P, T, B, Y>>,
    /// current frame poses
    pub poses: ReadStorage<'a, T>,
    /// next frame poses
    pub next_poses: ReadStorage<'a, NextFrame<T>>,
    /// entities
    pub entities: Entities<'a>,
    /// dirty poses
    pub dirty: &'a BitSet,
}

impl<'a, P, T, B, Y, D> CollisionData<Entity, P, T, B, Y, D>
    for SpatialCollisionData<'a, P, T, B, Y>
where
    P: Primitive + ComputeBound<B> + Send + Sync + 'static,
    P::Point: Debug + Send + Sync + 'static,
    <P::Point as EuclideanSpace>::Scalar: Send + Sync + 'static,
    <P::Point as EuclideanSpace>::Diff: Debug + Send + Sync + 'static,
    T: Component + Transform<P::Point> + Send + Sync + Clone + 'static,
    Y: Default + Send + Sync + 'static,
    B: Bound<Point = P::Point> + Send + Sync + 'static + Union<B, Output = B> + Clone,
{
    fn get_broad_data(&self) -> Vec<D> {
        Vec::default()
    }

    fn get_shape(&self, id: Entity) -> Option<&CollisionShape<P, T, B, Y>> {
        self.shapes.get(id)
    }

    fn get_pose(&self, id: Entity) -> Option<&T> {
        self.poses.get(id)
    }

    fn get_dirty_poses(&self) -> Vec<Entity> {
        (&*self.entities, self.dirty, &self.shapes)
            .join()
            .map(|(entity, _, _)| entity)
            .collect()
    }

    fn get_next_pose(&self, id: Entity) -> Option<&T> {
        self.next_poses.get(id).as_ref().map(|p| &p.value)
    }
}
