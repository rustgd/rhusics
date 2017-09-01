Physics library for use in Specs, using cgmath and collision-rs.

```rust
use cgmath::{Transform, Rotation2, Rad, Point2};
    use specs::{World, RunNow};

    use collide2d::*;

    type Shape = CollisionShape2D<BodyPose2D>;
    type Pose = BodyPose2D;
    type System = CollisionSystem2D<Pose>;

    #[test]
    pub fn test_world() {
        let mut world = World::new();
        world_register::<Pose>(&mut world);
        world
            .create_entity()
            .with(Shape::new_simple(CollisionStrategy::FullResolution, Rectangle::new(10., 10.).into(),))
            .with(Pose::one());
        world
            .create_entity()
            .with(Shape::new_simple(CollisionStrategy::FullResolution, Rectangle::new(10., 10.).into(),))
            .with(Pose::new(Point2::new(3., 2.), Rotation2::from_angle(Rad(0.))));

        let mut system = System::new()
            .with_broad_phase(BroadBruteForce2D::default())
            .with_narrow_phase(GJK2D::new());
        system.run_now(&world.res);
        println!("Contacts: {:?}", *world.read_resource::<Contacts2D>());
    }
```

NOTE: Currently only have working 2D collision detection.