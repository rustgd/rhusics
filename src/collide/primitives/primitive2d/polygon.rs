//! Convex polygon primitive

use cgmath::{Point2, Vector2};
use cgmath::prelude::*;
use collision::{Aabb2, Ray2, Line2};
use collision::prelude::*;

use {Pose, Real};
use collide::primitives::{HasAABB, SupportFunction, DiscreteTransformed, ContinuousTransformed};

/// Convex polygon primitive.
///
/// Can contain any number of vertices, but a high number of vertices will
/// affect performance of course. Vertices need to be in CCW order.
#[derive(Debug, Clone)]
pub struct ConvexPolygon {
    /// Vertices of the convex polygon.
    pub vertices: Vec<Point2<Real>>,
}

impl ConvexPolygon {
    /// Create a new convex polygon from the given vertices. Vertices need to be in CCW order.
    pub fn new(vertices: Vec<Point2<Real>>) -> Self {
        Self { vertices }
    }
}

impl SupportFunction for ConvexPolygon {
    type Point = Point2<Real>;

    fn support_point<T>(&self, direction: &Vector2<Real>, transform: &T) -> Point2<Real>
    where
        T: Pose<Point2<Real>>,
    {
        if self.vertices.len() < 10 {
            ::util::get_max_point(&self.vertices, direction, transform)
        } else {
            get_max_point(&self.vertices, direction, transform)
        }
    }
}

impl HasAABB for ConvexPolygon {
    type Aabb = Aabb2<Real>;

    fn get_bound(&self) -> Aabb2<Real> {
        ::util::get_bound(&self.vertices)
    }
}

impl DiscreteTransformed<Ray2<Real>> for ConvexPolygon {
    type Point = Point2<Real>;

    fn intersects_transformed<T>(&self, ray: &Ray2<Real>, transform: &T) -> bool
    where
        T: Transform<Point2<Real>>,
    {
        self.intersects(&ray.transform(transform.inverse_transform().unwrap()))
    }
}

impl Discrete<Ray2<Real>> for ConvexPolygon {
    /// Ray must be in object space
    fn intersects(&self, ray: &Ray2<Real>) -> bool {
        for j in 0..self.vertices.len() - 1 {
            let i = if j == 0 {
                self.vertices.len() - 1
            } else {
                j - 1
            };
            let normal = Vector2::new(
                self.vertices[j].y - self.vertices[i].y,
                self.vertices[i].x - self.vertices[j].x,
            );
            // check if edge normal points toward the ray origin
            if ray.direction.dot(normal) < 0. {
                // check line ray intersection
                match ray.intersection(&Line2::new(self.vertices[i], self.vertices[j])) {
                    Some(_) => return true,
                    _ => (),
                }
            }
        }

        false
    }
}

impl ContinuousTransformed<Ray2<Real>> for ConvexPolygon {
    type Point = Point2<Real>;
    type Result = Point2<Real>;

    fn intersection_transformed<T>(&self, ray: &Ray2<Real>, transform: &T) -> Option<Point2<Real>>
    where
        T: Transform<Point2<Real>>,
    {
        self.intersection(&ray.transform(transform.inverse_transform().unwrap()))
            .map(|p| transform.transform_point(p))
    }
}

impl Continuous<Ray2<Real>> for ConvexPolygon {
    type Result = Point2<Real>;

    /// Ray must be in object space
    fn intersection(&self, ray: &Ray2<Real>) -> Option<Point2<Real>> {
        for j in 0..self.vertices.len() - 1 {
            let i = if j == 0 {
                self.vertices.len() - 1
            } else {
                j - 1
            };
            let normal = Vector2::new(
                self.vertices[j].y - self.vertices[i].y,
                self.vertices[i].x - self.vertices[j].x,
            );
            // check if edge normal points toward the ray origin
            if ray.direction.dot(normal) < 0. {
                // check line ray intersection
                match ray.intersection(&Line2::new(self.vertices[i], self.vertices[j])) {
                    Some(point) => return Some(point),
                    _ => (),
                }
            }
        }

        None
    }
}

fn get_max_point<P, T>(vertices: &Vec<P>, direction: &P::Diff, transform: &T) -> P
where
    P: EuclideanSpace<Scalar = Real>,
    T: Pose<P>,
{
    let direction = transform.inverse_rotation().rotate_vector(*direction);

    // figure out where to start, if the direction is negative for the first vertex,
    // go halfway around the polygon
    let mut start_index: i32 = 0;
    let mut max_dot = vertices[0].dot(direction);
    if max_dot < P::Scalar::zero() {
        start_index = vertices.len() as i32 / 2;
        max_dot = dot_index(vertices, start_index, &direction);
    }

    let left_dot = dot_index(vertices, start_index - 1, &direction);
    let right_dot = dot_index(vertices, start_index + 1, &direction);

    // check if start is highest
    let p = if start_index == 0 && max_dot > left_dot && max_dot > right_dot {
        vertices[0]
    } else {
        // figure out iteration direction
        let mut add: i32 = 1;
        let mut previous_dot = left_dot;
        if left_dot > max_dot && left_dot > right_dot {
            add = -1;
            previous_dot = right_dot;
        }

        // iterate
        let mut index = start_index + add;
        let mut current_dot = max_dot;
        if index == vertices.len() as i32 {
            index = 0;
        }
        if index == -1 {
            index = vertices.len() as i32 - 1;
        }
        while index != start_index {
            let next_dot = dot_index(vertices, index + add, &direction);
            if current_dot > previous_dot && current_dot > next_dot {
                break;
            }
            previous_dot = current_dot;
            current_dot = next_dot;
            index += add;
            if index == vertices.len() as i32 {
                index = 0;
            }
            if index == -1 {
                index = vertices.len() as i32 - 1;
            }
        }
        vertices[index as usize]
    };

    *transform.position() + transform.rotation().rotate_point(p).to_vec()
}

#[inline]
fn dot_index<P>(vertices: &Vec<P>, index: i32, direction: &P::Diff) -> Real
where
    P: EuclideanSpace<Scalar = Real>,
{
    let index_u = index as usize;
    if index_u == vertices.len() {
        vertices[0].dot(*direction)
    } else if index == -1 {
        vertices[vertices.len() - 1].dot(*direction)
    } else {
        vertices[index_u].dot(*direction)
    }
}

#[cfg(test)]
mod tests {
    use cgmath::{Point2, Vector2};

    use super::*;
    use collide2d::BodyPose2;

    #[test]
    fn test_max_point() {
        let vertices = vec![
            Point2::new(5., 5.),
            Point2::new(4., 6.),
            Point2::new(3., 7.),
            Point2::new(2., 6.),
            Point2::new(1., 6.),
            Point2::new(0., 5.),
            Point2::new(-1., 4.),
            Point2::new(-3., 3.),
            Point2::new(-6., 1.),
            Point2::new(-5., 0.),
            Point2::new(-4., -1.),
            Point2::new(-2., -3.),
            Point2::new(0., -7.),
            Point2::new(1., -8.),
            Point2::new(2., -5.),
            Point2::new(3., 0.),
            Point2::new(4., 3.),
        ];
        let transform = BodyPose2::one();
        let point = get_max_point(&vertices, &Vector2::new(-1., 0.), &transform);
        assert_eq!(Point2::new(-5., 0.), point);

        let point = get_max_point(&vertices, &Vector2::new(0., -1.), &transform);
        assert_eq!(Point2::new(1., -8.), point);

        let point = get_max_point(&vertices, &Vector2::new(0., 1.), &transform);
        assert_eq!(Point2::new(3., 7.), point);

        let point = get_max_point(&vertices, &Vector2::new(1., 0.), &transform);
        assert_eq!(Point2::new(5., 5.), point);
    }
}
