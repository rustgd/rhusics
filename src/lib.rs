extern crate cgmath;
extern crate collision;
extern crate specs;

#[macro_use]
extern crate log;

#[macro_use]
extern crate specs_derive;

//use specs::World;

pub mod two;

/*
use components::{Body, BodyPose, Shape};

pub struct Physics {
    world : World,
    time_step : f32,
    max_iterations : u32
}

impl Physics {

    pub fn new(time_step : f32, max_iterations : u32) -> Physics {
        let mut world = World::new();
        world.register::<Body>();
        world.register::<BodyPose>();
        world.register::<Shape>();
        Physics {
            world : world,
            time_step : time_step,
            max_iterations : max_iterations
        }
    }

    pub fn update(&mut self, dt : f32) {
        let bodies = self.world.read::<Body>();

        // calculate unconstrained velocity and position
        // do collision detection
          // broad phase
          // narrow phase
          // generate manifolds
        // solve for constraint impulse
        // calculate constrained velocity and position
        // report collision events
    }
}*/