use std::ops::Neg;

use cgmath::{Vector2, Point2};
use cgmath::prelude::*;

use super::SimplexProcessor;
use Real;
use collide::narrow::gjk::SupportPoint;

pub struct SimplexProcessor2D;

impl SimplexProcessor for SimplexProcessor2D {
    type Vector = Vector2<Real>;
    type Point = Point2<Real>;

    fn check_origin(
        &self,
        simplex: &mut Vec<SupportPoint<Point2<Real>>>,
        d: &mut Vector2<Real>,
    ) -> bool {
        // 3 points
        if simplex.len() == 3 {
            let a = simplex[2].v;
            let b = simplex[1].v;
            let c = simplex[0].v;

            let ao = a.neg();
            let ab = b - a;
            let ac = c - a;

            let abp = ::util::triple_product(&ac, &ab, &ab);
            if abp.dot(ao) > 0. {
                simplex.remove(0);
                *d = abp;
            } else {
                let acp = ::util::triple_product(&ab, &ac, &ac);
                if acp.dot(ao) > 0. {
                    simplex.remove(1);
                    *d = acp;
                } else {
                    return true;
                }
            }
        }
        // 2 points
        else if simplex.len() == 2 {
            let a = simplex[1].v;
            let b = simplex[0].v;

            let ao = a.neg();
            let ab = b - a;

            *d = ::util::triple_product(&ab, &ao, &ab);
        }
        // 0-1 point means we can't really do anything
        false
    }

    fn new() -> Self {
        Self {}
    }
}

#[cfg(test)]
mod tests {
    use cgmath::Vector2;

    use super::*;
    use collide::narrow::gjk::SupportPoint;

    #[test]
    fn test_check_origin_empty() {
        let processor = SimplexProcessor2D::new();
        let mut direction = Vector2::new(1., 0.);
        let mut simplex = vec![];
        assert!(!processor.check_origin(&mut simplex, &mut direction));
        assert_eq!(0, simplex.len());
        assert_eq!(Vector2::new(1., 0.), direction);
    }

    #[test]
    fn test_check_origin_single() {
        let processor = SimplexProcessor2D::new();
        let mut direction = Vector2::new(1., 0.);
        let mut simplex = vec![sup(40., 0.)];
        assert!(!processor.check_origin(&mut simplex, &mut direction));
        assert_eq!(1, simplex.len());
        assert_eq!(Vector2::new(1., 0.), direction);
    }

    #[test]
    fn test_check_origin_edge() {
        let processor = SimplexProcessor2D::new();
        let mut direction = Vector2::new(1., 0.);
        let mut simplex = vec![sup(40., 10.), sup(-10., 10.)];
        assert!(!processor.check_origin(&mut simplex, &mut direction));
        assert_eq!(2, simplex.len());
        assert_eq!(0., direction.x);
        assert!(direction.y < 0.);
    }

    #[test]
    fn test_check_origin_triangle_outside_ac() {
        let processor = SimplexProcessor2D::new();
        let mut direction = Vector2::new(1., 0.);
        let mut simplex = vec![
            sup(40., 10.),
            sup(-10., 10.),
            sup(0., 3.),
        ];
        assert!(!processor.check_origin(&mut simplex, &mut direction));
        assert_eq!(2, simplex.len());
        assert!(direction.x < 0.);
        assert!(direction.y < 0.);
    }

    #[test]
    fn test_check_origin_triangle_outside_ab() {
        let processor = SimplexProcessor2D::new();
        let mut direction = Vector2::new(1., 0.);
        let mut simplex = vec![
            sup(40., 10.),
            sup(10., 10.),
            sup(3., -3.),
        ];
        assert!(!processor.check_origin(&mut simplex, &mut direction));
        assert_eq!(2, simplex.len());
        assert!(direction.x < 0.);
        assert!(direction.y > 0.);
    }

    #[test]
    fn test_check_origin_triangle_hit() {
        let processor = SimplexProcessor2D::new();
        let mut direction = Vector2::new(1., 0.);
        let mut simplex = vec![
            sup(40., 10.),
            sup(-10., 10.),
            sup(0., -3.),
        ];
        assert!(processor.check_origin(&mut simplex, &mut direction));
        assert_eq!(3, simplex.len());
        assert_eq!(Vector2::new(1., 0.), direction);
    }

    fn sup(x: f32, y: f32) -> SupportPoint<Point2<Real>> {
        let mut s = SupportPoint::new();
        s.v = Vector2::new(x, y);
        s
    }
}
