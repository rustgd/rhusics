use std::collections::{HashMap, HashSet};
use std::fmt::Debug;
use std::marker::PhantomData;

use cgmath::BaseFloat;
use cgmath::prelude::*;
use collision::dbvt::{DynamicBoundingVolumeTree, TreeValue};
use collision::prelude::*;
use specs::{Component, Entities, Entity, FetchMut, Join, ReadStorage, System, WriteStorage};

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
    marker: PhantomData<(P, T, Y, B, D)>,
}

impl<P, T, D, B, Y> SpatialSortingSystem<P, T, D, B, Y> {
    /// Create a new sorting system.
    pub fn new() -> Self {
        Self {
            entities: HashMap::default(),
            marker: PhantomData,
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
    Y: Default + Send + Sync + 'static,
    for<'b> &'b T::Storage: Join<Type = &'b T>,
    D: Send + Sync + 'static + TreeValue<Bound = B> + From<(Entity, B)>,
{
    type SystemData = (
        Entities<'a>,
        ReadStorage<'a, T>,
        ReadStorage<'a, NextFrame<T>>,
        WriteStorage<'a, CollisionShape<P, T, B, Y>>,
        FetchMut<'a, DynamicBoundingVolumeTree<D>>,
    );

    fn run(&mut self, (entities, poses, next_poses, mut shapes, mut tree): Self::SystemData) {
        let mut keys = self.entities.keys().cloned().collect::<HashSet<Entity>>();

        // Check for updated poses that are already in the tree
        // Uses FlaggedStorage
        for (entity, pose, shape) in (&*entities, (&poses).open().1, &mut shapes).join() {
            shape.update(pose, None);

            // Update the wrapper in the tree for the shape
            if let Some(node_index) = self.entities.get(&entity).cloned() {
                tree.update_node(node_index, (entity, shape.bound().clone()).into());
            }
        }

        // Check for updated next frame poses that are already in the tree
        // Uses FlaggedStorage
        for (entity, pose, next_pose, shape) in
            (&*entities, &poses, (&next_poses).open().1, &mut shapes).join()
        {
            shape.update(pose, Some(&next_pose.value));

            // Update the wrapper in the tree for the shape
            if let Some(node_index) = self.entities.get(&entity).cloned() {
                tree.update_node(node_index, (entity, shape.bound().clone()).into());
            }
        }

        // For all active shapes, remove them from the deletion list, and add any new entities
        // to the tree.
        for (entity, _, shape) in (&*entities, &poses, &shapes).join() {
            // entity still exists, remove from deletion list
            keys.remove(&entity);

            // if entity does not exist in entities list, add it to the tree and entities list
            if self.entities.get(&entity).is_none() {
                let node_index = tree.insert((entity, shape.bound().clone()).into());
                self.entities.insert(entity, node_index);
            }
        }

        // remove entities that are missing from the tree
        for entity in keys {
            if let Some(node_index) = self.entities.get(&entity).cloned() {
                tree.remove(node_index);
                self.entities.remove(&entity);
            }
        }

        // process possibly updated values
        tree.update();

        // do refitting
        tree.do_refit();
    }
}
