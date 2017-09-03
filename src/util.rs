use cgmath::{Vector2, Zero};
use cgmath::num_traits::Float;
use cgmath::prelude::*;
use collision::Aabb;

use {Pose, Real};

pub(crate) fn get_max_point<P, T>(vertices: &Vec<P>, direction: &P::Diff, transform: &T) -> P
where
    P: EuclideanSpace<Scalar = Real>,
    T: Pose<P>,
{
    let direction = transform.inverse_rotation().rotate_vector(*direction);
    let (p, _) = vertices.iter().map(|v| (v, v.dot(direction))).fold(
        (
            P::from_value(P::Scalar::zero()),
            P::Scalar::neg_infinity(),
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
    *transform.position() + transform.rotation().rotate_point(p).to_vec()
}

pub(crate) fn get_bound<A>(vertices: &Vec<A::Point>) -> A
where
    A: Aabb,
{
    vertices.iter().fold(A::zero(), |bound, p| bound.grow(*p))
}

#[inline]
pub(crate) fn triple_product(a: &Vector2<Real>, b: &Vector2<Real>, c: &Vector2<Real>) -> Vector2<Real> {
    let ac = a.x * c.x + a.y * c.y;
    let bc = b.x * c.x + b.y * c.y;
    Vector2::new(b.x * ac - a.x * bc, b.y * ac - a.y * bc)
}

#[cfg(test)]
mod tests {
    use std;

    use cgmath::{Vector2, Rotation2, Rad, Point2, Basis2};
    use collision::Aabb2;

    use super::*;
    use {BodyPose, Real};

    #[test]
    fn test_get_bound() {
        let triangle = vec![
            Point2::new(-1., 1.),
            Point2::new(0., -1.),
            Point2::new(1., 0.),
        ];
        assert_eq!(
            Aabb2::new(Point2::new(-1., -1.), Point2::new(1., 1.)),
            get_bound(&triangle)
        );
    }

    fn test_max_point(dx: Real, dy: Real, px: Real, py: Real, rot_angle: Real) {
        let direction = Vector2::new(dx, dy);
        let point = Point2::new(px, py);
        let triangle = vec![
            Point2::new(-1., 1.),
            Point2::new(0., -1.),
            Point2::new(1., 0.),
        ];
        let transform: BodyPose<Point2<Real>, Basis2<Real>> =
            BodyPose::new(Point2::new(0., 0.), Rotation2::from_angle(Rad(rot_angle)));
        let max_point = get_max_point(&triangle, &direction, &transform);
        assert_approx_eq!(point.x, max_point.x);
        assert_approx_eq!(point.y, max_point.y);
    }

    #[test]
    fn test_max_point_1() {
        test_max_point(0., 1., -1., 1., 0.);
    }

    #[test]
    fn test_max_point_2() {
        test_max_point(-1., 0., -1., 1., 0.);
    }

    #[test]
    fn test_max_point_3() {
        test_max_point(0., -1., 0., -1., 0.);
    }

    #[test]
    fn test_max_point_4() {
        test_max_point(1., 0., 1., 0., 0.);
    }

    #[test]
    fn test_max_point_5() {
        test_max_point(10., 1., 1., 0., 0.);
    }

    #[test]
    fn test_max_point_6() {
        test_max_point(2., -100., 0., -1., 0.);
    }

    #[test]
    fn test_max_point_rot() {
        test_max_point(
            0.,
            1.,
            std::f64::consts::FRAC_1_SQRT_2 as Real,
            std::f64::consts::FRAC_1_SQRT_2 as Real,
            std::f64::consts::PI as Real / 4.,
        );
    }

    #[test]
    fn test_max_point_disp() {
        let direction = Vector2::new(0., 1.);
        let point = Point2::new(-1., 9.);
        let triangle = vec![
            Point2::new(-1., 1.),
            Point2::new(0., -1.),
            Point2::new(1., 0.),
        ];
        let transform: BodyPose<Point2<Real>, Basis2<Real>> =
            BodyPose::new(Point2::new(0., 8.), Rotation2::from_angle(Rad(0.)));
        assert_eq!(point, get_max_point(&triangle, &direction, &transform));
    }
}
