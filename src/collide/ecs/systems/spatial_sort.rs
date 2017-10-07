use std::collections::{HashMap, HashSet};
use std::fmt::Debug;
use std::marker::PhantomData;

use cgmath::prelude::*;
use collision::dbvt::DynamicBoundingVolumeTree;
use collision::prelude::*;
use specs::{Component, Entities, Entity, FetchMut, Join, ReadStorage, System, WriteStorage};

use Real;
use collide::{CollisionShape, ContainerShapeWrapper, Primitive};

/// Spatial sorting [system](https://docs.rs/specs/0.9.5/specs/trait.System.html) for use with
/// [`specs`](https://docs.rs/specs/0.9.5/specs/).
///
/// Will perform spatial sorting of the collision world. Uses a Dynamic Bounding Volume Tree for
/// sorting. Will update entries in the tree where the pose is dirty.
///
/// Can handle any transform component type, as long as the type implements
/// [`Pose`](../../trait.Pose.html) and
/// [`Transform`](https://docs.rs/cgmath/0.15.0/cgmath/trait.Transform.html).
///
#[derive(Debug)]
pub struct SpatialSortingSystem<P, T> {
    entities: HashMap<Entity, usize>,
    marker: PhantomData<(P, T)>,
}

impl<P, T> SpatialSortingSystem<P, T> {
    /// Create a new collision detection system, with no broad or narrow phase activated.
    pub fn new() -> Self {
        Self {
            entities: HashMap::default(),
            marker: PhantomData,
        }
    }
}

impl<'a, P, T> System<'a> for SpatialSortingSystem<P, T>
where
    P: Primitive + Send + Sync + 'static,
    P::Aabb: Clone
        + Debug
        + Send
        + Sync
        + Aabb<Scalar = Real>
        + Contains<P::Aabb>
        + SurfaceArea<Scalar = Real>,
    P::Point: Debug,
    <P::Point as EuclideanSpace>::Diff: Debug + Send + Sync,
    T: Component + Clone + Debug + Transform<P::Point> + Send + Sync,
    for <'b : 'a> &'b T::Storage: Join<Type=T>,
{
    type SystemData = (
        Entities<'a>,
        ReadStorage<'a, T>,
        WriteStorage<'a, CollisionShape<P, T>>,
        FetchMut<'a, DynamicBoundingVolumeTree<ContainerShapeWrapper<Entity, P>>>,
    );

    fn run(&mut self, (entities, poses, mut shapes, mut tree): Self::SystemData) {
        let mut keys = self.entities.keys().cloned().collect::<HashSet<Entity>>();

        for (entity, pose, shape) in (&*entities, (&poses).open().1, &mut shapes).join() {
            shape.update(&pose);

            match self.entities.get(&entity).cloned() {
                Some(node_index) => tree.update_node(
                    node_index,
                    ContainerShapeWrapper::new(entity, shape.bound()),
                ),

                _ => (),
            }
        }

        for (entity, _, shape) in (&*entities, &poses, &shapes).join() {
            // entity still exists, remove from deletion list
            keys.remove(&entity);

            match self.entities.get(&entity) {
                // entity exists in tree, possibly update it with new values
                Some(_) => (),

                // entity does not exist in tree, add it to the tree and entities map
                None => {
                    let node_index = tree.insert(ContainerShapeWrapper::new(entity, shape.bound()));
                    self.entities.insert(entity, node_index);
                }
            }
        }

        // remove entities that are missing from the tree
        for entity in keys {
            let node_index = match self.entities.get(&entity) {
                Some(node_index) => Some(node_index.clone()),
                None => None,
            };
            match node_index {
                Some(node_index) => {
                    tree.remove(node_index);
                    self.entities.remove(&entity);
                }
                None => (),
            }
        }

        // process possibly updated values
        tree.update();

        // do refitting
        tree.do_refit();
    }
}
