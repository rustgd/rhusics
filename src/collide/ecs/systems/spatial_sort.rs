use std::collections::{HashMap, HashSet};
use std::fmt::Debug;
use std::marker::PhantomData;

use collision::dbvt::DynamicBoundingVolumeTree;
use collision::prelude::*;
use specs::{Component, Entities, Entity, FetchMut, Join, ReadStorage, System, WriteStorage};

use {Pose, Real};
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
        + 'static
        + Contains<P::Aabb>
        + SurfaceArea<Scalar = Real>,
    P::Vector: Debug + Send + Sync + 'static,
    T: Component
        + Clone
        + Debug
        + Pose<P::Point>
        + Send
        + Sync
        + 'static,
{
    type SystemData = (Entities<'a>,
     ReadStorage<'a, T>,
     WriteStorage<'a, CollisionShape<P, T>>,
     FetchMut<'a, DynamicBoundingVolumeTree<ContainerShapeWrapper<Entity, P>>>);

    fn run(&mut self, (entities, poses, mut shapes, mut tree): Self::SystemData) {
        let mut keys = self.entities.keys().cloned().collect::<HashSet<Entity>>();
        for (entity, pose, shape) in (&*entities, &poses, &mut shapes).join() {
            // update the bound in the shape
            if pose.dirty() {
                shape.update(&pose);
            }

            // entity still exists, remove from deletion list
            keys.remove(&entity);

            let node_index = match self.entities.get(&entity) {
                Some(node_index) => Some(node_index.clone()),
                None => None,
            };
            match node_index {
                // entity exists in tree, possibly update it with new values
                Some(node_index) => {
                    if pose.dirty() {
                        tree.update_node(
                            node_index,
                            ContainerShapeWrapper::new(entity, shape.bound()),
                        );
                    }
                }

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
