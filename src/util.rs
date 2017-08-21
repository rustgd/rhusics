use cgmath::prelude::*;
use cgmath::{BaseFloat, Vector2, Decomposed};
use collision::{Aabb, MinMax};

pub fn get_max_point<S, V, P, R>(
    vertices: &Vec<V>,
    direction: &V,
    transform: &Decomposed<V, R>,
) -> P
where
    S: BaseFloat,
    V: VectorSpace<Scalar = S> + ElementWise + Array<Element = S> + InnerSpace,
    P: EuclideanSpace<Scalar = S, Diff = V>,
    R: Rotation<P>,
{
    let direction = transform.rot.invert().rotate_vector(*direction);
    let (p, _) = vertices.iter().map(|v| (v, v.dot(direction))).fold(
        (
            V::zero(),
            S::neg_infinity(),
        ),
        |(max_p,
          max_dot),
         (v, dot)| {
            if dot > max_dot {
                (v.clone(), dot)
            } else {
                (max_p, max_dot)
            }
        },
    );
    transform.transform_point(P::from_vec(p))
}

pub fn get_bound<S, V, P, A>(vertices: &Vec<V>) -> A
where
    S: BaseFloat,
    V: VectorSpace<Scalar = S> + ElementWise + Array<Element = S>,
    P: EuclideanSpace<Scalar = S, Diff = V> + MinMax,
    A: Aabb<S, V, P>,
{
    vertices.iter().fold(
        A::new(P::from_value(S::zero()), P::from_value(S::zero())),
        |bound, p| bound.grow(P::from_vec(*p)),
    )
}

#[inline]
pub fn triple_product<S: BaseFloat>(a: &Vector2<S>, b: &Vector2<S>, c: &Vector2<S>) -> Vector2<S> {
    let ac = a.x * c.x + a.y * c.y;
    let bc = b.x * c.x + b.y * c.y;
    Vector2::new(b.x * ac - a.x * bc, b.y * ac - a.y * bc)
}

#[cfg(test)]
mod tests {
    use std;
    use super::*;
    use cgmath::{Decomposed, Vector2, Rotation2, Rad, Point2, Basis2};
    use collision::Aabb2;

    #[test]
    fn test_get_bound() {
        let triangle = vec![
            Vector2::new(-1., 1.),
            Vector2::new(0., -1.),
            Vector2::new(1., 0.),
        ];
        assert_eq!(Aabb2::new(Point2::new(-1., -1.), Point2::new(1., 1.)),
                   get_bound(&triangle));
    }

    fn test_max_point(dx : f32, dy: f32, px: f32, py: f32, rot_angle: f32) {
        let direction = Vector2::new(dx, dy);
        let point = Point2::new(px, py);
        let triangle = vec![
            Vector2::new(-1., 1.),
            Vector2::new(0., -1.),
            Vector2::new(1., 0.),
        ];
        let transform = Decomposed::<Vector2<f32>, Basis2<f32>> {
            disp: Vector2::new(0., 0.),
            rot: Rotation2::from_angle(Rad(rot_angle)),
            scale: 1.,
        };
        assert_eq!(
            point,
            get_max_point(&triangle, &direction, &transform)
        );
    }

    #[test]
    fn test_max_point_1() {
        test_max_point(0., 1.,-1., 1., 0.);
    }

    #[test]
    fn test_max_point_2() {
        test_max_point(-1., 0.,-1., 1., 0.);
    }

    #[test]
    fn test_max_point_3() {
        test_max_point(0., -1.,0., -1., 0.);
    }

    #[test]
    fn test_max_point_4() {
        test_max_point(1., 0.,1., 0., 0.);
    }

    #[test]
    fn test_max_point_5() {
        test_max_point(10., 1.,1., 0., 0.);
    }

    #[test]
    fn test_max_point_6() {
        test_max_point(2., -100.,0., -1., 0.);
    }

    #[test]
    fn test_max_point_rot() {
        test_max_point(0., 1., 0.70710677, 0.70710677, std::f32::consts::PI / 4.);
    }

    #[test]
    fn test_max_point_disp() {
        let direction = Vector2::new(0., 1.);
        let point = Point2::new(-1., 9.);
        let triangle = vec![
            Vector2::new(-1., 1.),
            Vector2::new(0., -1.),
            Vector2::new(1., 0.),
        ];
        let transform = Decomposed::<Vector2<f32>, Basis2<f32>> {
            disp: Vector2::new(0., 8.),
            rot: Rotation2::from_angle(Rad(0.)),
            scale: 1.,
        };
        assert_eq!(
            point,
            get_max_point(&triangle, &direction, &transform)
        );
    }
}
