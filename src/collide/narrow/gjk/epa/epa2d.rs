use cgmath::{Vector2, Point2};
use cgmath::num_traits::Float;
use collision::Aabb2;

use super::*;
use {Pose, Real};
use collide::{CollisionPrimitive, Contact, CollisionStrategy};
use collide::narrow::gjk::support;
use collide::primitive2d::Primitive2D;

pub struct EPA2D;

impl<T> EPA<T> for EPA2D
where
    T: Pose<Point2<Real>>,
{
    type Vector = Vector2<Real>;
    type Point = Point2<Real>;
    type Aabb = Aabb2<Real>;
    type Primitive = Primitive2D;

    fn process(
        &self,
        simplex: &mut Vec<SupportPoint<Point2<Real>>>,
        left: &CollisionPrimitive<Primitive2D, Aabb2<Real>, T>,
        left_transform: &T,
        right: &CollisionPrimitive<Primitive2D, Aabb2<Real>, T>,
        right_transform: &T,
    ) -> Vec<Contact<Vector2<Real>>> {
        let mut i = 0;
        if closest_edge(&simplex).is_none() {
            return Vec::default();
        }

        loop {
            let e = closest_edge(&simplex);
            let e = e.unwrap();
            let p = support(left, left_transform, right, right_transform, &e.normal);
            let d = p.v.dot(e.normal);
            if d - e.distance < EPA_TOLERANCE {
                return vec![
                    Contact::new_impl(
                        CollisionStrategy::FullResolution,
                        e.normal,
                        e.distance
                    ),
                ];
            } else {
                simplex.insert(e.index, p);
            }
            i += 1;
            if i >= MAX_ITERATIONS {
                return vec![
                    Contact::new_impl(
                        CollisionStrategy::FullResolution,
                        e.normal,
                        e.distance
                    ),
                ];
            }
        }
    }

    fn new() -> Self {
        Self {}
    }
}

#[derive(Debug)]
struct Edge {
    pub normal: Vector2<Real>,
    pub distance: Real,
    pub index: usize,
}

impl Edge {
    pub fn new() -> Self {
        Self {
            normal: Vector2::zero(),
            distance: Real::infinity(),
            index: 0,
        }
    }
}

fn closest_edge(simplex: &Vec<SupportPoint<Point2<Real>>>) -> Option<Edge> {
    if simplex.len() < 3 {
        None
    } else {
        let mut edge = Edge::new();
        for i in 0..simplex.len() {
            let j = if i + 1 == simplex.len() { 0 } else { i + 1 };
            let a = simplex[i].v;
            let b = simplex[j].v;
            let e = b - a;
            let oa = a;
            let n = ::util::triple_product(&e, &oa, &e).normalize();
            let d = n.dot(a);
            if d < edge.distance {
                edge.distance = d;
                edge.normal = n;
                edge.index = j;
            }
        }
        Some(edge)
    }
}

#[cfg(test)]
mod tests {

    use cgmath::{Point2, Vector2, Rotation2, Rad};

    use super::*;
    use collide::narrow::gjk::SupportPoint;

    use Real;
    use collide2d::*;

    #[test]
    fn test_closest_edge_0() {
        assert!(closest_edge(&vec![]).is_none())
    }

    #[test]
    fn test_closest_edge_1() {
        assert!(
        closest_edge(&vec![sup(10., 10.)])
            .is_none()
        )
    }

    #[test]
    fn test_closest_edge_2() {
        assert!(
        closest_edge(&vec![sup(10., 10.), sup(-10., 5.)])
            .is_none()
        )
    }

    #[test]
    fn test_closest_edge_3() {
        let edge = closest_edge(&vec![
            sup(10., 10.),
            sup(-10., 5.),
            sup(2., -5.),
        ]);
        assert!(edge.is_some());
        let edge = edge.unwrap();
        assert_eq!(2, edge.index);
        assert_approx_eq!(2.5607374, edge.distance);
        assert_approx_eq!(-0.6401844, edge.normal.x);
        assert_approx_eq!(-0.7682213, edge.normal.y);
    }

    // let left = CollisionPrimitive2D::new(Rectangle::new(10., 10.).into());
    // let left_transform = transform(15., 0., 0.);
    // let right = CollisionPrimitive2D::new(Rectangle::new(10., 10.).into());
    // let right_transform = transform(7., 2., 0.);
    // let mut simplex = vec![
    //  SupportPoint { v: Vector2 [-2, 8], sup_a: Point2 [10, 5], sup_b: Point2 [12, -3] },
    //  SupportPoint { v: Vector2 [18, -12], sup_a: Point2 [20, -5], sup_b: Point2 [2, 7] },
    //  SupportPoint { v: Vector2 [-2, -12], sup_a: Point2 [10, -5], sup_b: Point2 [12, 7] }
    // ]

    #[test]
    fn test_epa_0() {
        let left = CollisionPrimitive2D::new(Rectangle::new(10., 10.).into());
        let left_transform = transform(15., 0., 0.);
        let right = CollisionPrimitive2D::new(Rectangle::new(10., 10.).into());
        let right_transform = transform(7., 2., 0.);
        let epa = EPA2D;
        assert!(epa.process(&mut vec![], &left, &left_transform, &right, &right_transform).is_empty());
    }

    #[test]
    fn test_epa_1() {
        let left = CollisionPrimitive2D::new(Rectangle::new(10., 10.).into());
        let left_transform = transform(15., 0., 0.);
        let right = CollisionPrimitive2D::new(Rectangle::new(10., 10.).into());
        let right_transform = transform(7., 2., 0.);
        let mut simplex = vec![
            sup(-2. , 8.)
        ];
        let epa = EPA2D;
        assert!(epa.process(&mut simplex, &left, &left_transform, &right, &right_transform).is_empty());
    }

    #[test]
    fn test_epa_2() {
        let left = CollisionPrimitive2D::new(Rectangle::new(10., 10.).into());
        let left_transform = transform(15., 0., 0.);
        let right = CollisionPrimitive2D::new(Rectangle::new(10., 10.).into());
        let right_transform = transform(7., 2., 0.);
        let mut simplex = vec![
            sup(-2. , 8.),
            sup(18., -12.),
        ];
        let epa = EPA2D;
        assert!(epa.process(&mut simplex, &left, &left_transform, &right, &right_transform).is_empty());
    }

    #[test]
    fn test_epa_3() {
        let left = CollisionPrimitive2D::new(Rectangle::new(10., 10.).into());
        let left_transform = transform(15., 0., 0.);
        let right = CollisionPrimitive2D::new(Rectangle::new(10., 10.).into());
        let right_transform = transform(7., 2., 0.);
        let mut simplex = vec![
            sup(-2. , 8.),
            sup(18., -12.),
            sup(-2., -12.),
        ];
        let epa = EPA2D;
        let contacts = epa.process(&mut simplex, &left, &left_transform, &right, &right_transform);
        assert_eq!(1, contacts.len());
        assert_eq!(Vector2::new(-1., 0.), contacts[0].normal);
        assert_eq!(2., contacts[0].penetration_depth);
    }

    fn sup(x: f32, y: f32) -> SupportPoint<Point2<Real>> {
        let mut s = SupportPoint::new();
        s.v = Vector2::new(x, y);
        s
    }

    fn transform(x: Real, y: Real, angle: Real) -> BodyPose2D {
        BodyPose2D::new(Point2::new(x, y), Rotation2::from_angle(Rad(angle)))
    }
}