use std::collections::HashMap;
use std::fmt::Debug;
use std::marker::PhantomData;

use cgmath::prelude::*;
use cgmath::BaseFloat;
use collision::dbvt::{DynamicBoundingVolumeTree, TreeValue};
use collision::prelude::*;
use specs::prelude::{
    BitSet, Component, ComponentEvent, Entities, Entity, Join, ReadStorage, ReaderId, World,
    System, Tracked, Write, WriteStorage,
};

use core::{CollisionShape, NextFrame, Primitive};

/// Spatial sorting [system](https://docs.rs/specs/0.9.5/specs/trait.System.html) for use with
/// [`specs`](https://docs.rs/specs/0.9.5/specs/).
///
/// Will perform spatial sorting of the collision world. Uses a Dynamic Bounding Volume Tree for
/// sorting. Will update entries in the tree where the pose is dirty.
///
/// Can handle any transform component type, as long as the type implements
/// [`Transform`](https://docs.rs/cgmath/0.15.0/cgmath/trait.Transform.html), and as long as the
/// storage is wrapped in
/// [`FlaggedStorage`](https://docs.rs/specs/0.9.5/specs/struct.FlaggedStorage.html)
///
/// ## Type parameters:
///
/// - `P`: Primitive type, needs to implement `Primitive`.
/// - `T`: Transform type, needs to implement `Transform` and have `FlaggedStorage`.
/// - `D`: Type of values stored in the DBVT, needs to implement `TreeValue` and
///        `From<(Entity, CollisionShape)>`
/// - `B`: Bounding volume
/// - `Y`: Shape type, see `Collider`
///
/// ### System Function:
///
/// `fn(Entities, T, NextFrame<T>, CollisionShape) -> (CollisionShape, DynamicBoundingVolumeTree<D>)`
#[derive(Debug)]
pub struct SpatialSortingSystem<P, T, D, B, Y = ()> {
    entities: HashMap<Entity, usize>,
    dead: Vec<Entity>,
    updated: BitSet,
    removed: BitSet,
    pose_reader: Option<ReaderId<ComponentEvent>>,
    next_pose_reader: Option<ReaderId<ComponentEvent>>,
    marker: PhantomData<(P, T, Y, B, D)>,
}

impl<P, T, D, B, Y> SpatialSortingSystem<P, T, D, B, Y> {
    /// Create a new sorting system.
    pub fn new() -> Self {
        SpatialSortingSystem {
            entities: HashMap::default(),
            marker: PhantomData,
            updated: BitSet::default(),
            removed: BitSet::default(),
            pose_reader: None,
            next_pose_reader: None,
            dead: Vec::default(),
        }
    }
}

impl<'a, P, T, Y, D, B> System<'a> for SpatialSortingSystem<P, T, D, B, Y>
where
    P: Primitive + ComputeBound<B> + Send + Sync + 'static,
    B: Clone
        + Debug
        + Send
        + Sync
        + Union<B, Output = B>
        + Bound<Point = P::Point>
        + Contains<B>
        + SurfaceArea<Scalar = <P::Point as EuclideanSpace>::Scalar>
        + Send
        + Sync
        + 'static,
    P::Point: Debug,
    <P::Point as EuclideanSpace>::Scalar: BaseFloat + Send + Sync + 'static,
    <P::Point as EuclideanSpace>::Diff: Debug + Send + Sync,
    T: Component + Clone + Debug + Transform<P::Point> + Send + Sync,
    T::Storage: Tracked,
    Y: Default + Send + Sync + 'static,
    D: Send + Sync + 'static + TreeValue<Bound = B> + From<(Entity, B)>,
{
    type SystemData = (
        Entities<'a>,
        ReadStorage<'a, T>,
        ReadStorage<'a, NextFrame<T>>,
        WriteStorage<'a, CollisionShape<P, T, B, Y>>,
        Write<'a, DynamicBoundingVolumeTree<D>>,
    );

    fn run(&mut self, (entities, poses, next_poses, mut shapes, mut tree): Self::SystemData) {
        self.updated.clear();
        self.removed.clear();

        for event in poses.channel().read(self.pose_reader.as_mut().unwrap()) {
            match event {
                ComponentEvent::Inserted(index) => {
                    self.updated.add(*index);
                }
                ComponentEvent::Modified(index) => {
                    self.updated.add(*index);
                }
                ComponentEvent::Removed(index) => {
                    self.updated.remove(*index);
                    self.removed.add(*index);
                }
            }
        }

        // remove entities that are missing from the tree
        self.dead.clear();
        for (entity, node_index) in &self.entities {
            if self.removed.contains(entity.id()) {
                tree.remove(*node_index);
                self.dead.push(*entity);
            }
        }
        for entity in &self.dead {
            self.entities.remove(entity);
        }

        // Check for updated poses
        // Uses FlaggedStorage
        for (entity, pose, shape, _) in (&*entities, &poses, &mut shapes, &self.updated).join() {
            shape.update(pose, None);

            // Update the wrapper in the tree for the shape
            match self.entities.get(&entity).cloned() {
                // Update existing
                Some(node_index) => {
                    tree.update_node(node_index, (entity, shape.bound().clone()).into());
                }
                // Insert new
                None => {
                    let node_index = tree.insert((entity, shape.bound().clone()).into());
                    self.entities.insert(entity, node_index);
                }
            }
        }

        self.updated.clear();

        for event in next_poses
            .channel()
            .read(self.next_pose_reader.as_mut().unwrap())
        {
            match event {
                ComponentEvent::Inserted(index) => {
                    self.updated.add(*index);
                }
                ComponentEvent::Modified(index) => {
                    self.updated.add(*index);
                }
                ComponentEvent::Removed(index) => {
                    self.updated.remove(*index);
                }
            }
        }

        // Check for updated next frame poses
        // Uses FlaggedStorage
        for (entity, pose, next_pose, shape, _) in
            (&*entities, &poses, &next_poses, &mut shapes, &self.updated).join()
        {
            shape.update(pose, Some(&next_pose.value));

            // Update the wrapper in the tree for the shape
            if let Some(node_index) = self.entities.get(&entity).cloned() {
                tree.update_node(node_index, (entity, shape.bound().clone()).into());
            }
        }

        // process possibly updated values
        tree.update();

        // do refitting
        tree.do_refit();
    }

    fn setup(&mut self, res: &mut World) {
        use specs::prelude::SystemData;
        Self::SystemData::setup(res);
        let mut poses = WriteStorage::<T>::fetch(res);
        self.pose_reader = Some(poses.register_reader());
        let mut next_poses = WriteStorage::<NextFrame<T>>::fetch(res);
        self.next_pose_reader = Some(next_poses.register_reader());
    }
}
