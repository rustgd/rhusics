use std::ops::Neg;

use cgmath::prelude::*;
use cgmath::{Vector2};

use Real;
use super::{SimplexProcessor, Feature};

pub struct SimplexProcessor2D;

impl SimplexProcessor for SimplexProcessor2D {
    type Vector = Vector2<Real>;

    fn check_origin(&self, simplex: &mut Vec<Vector2<Real>>, d: &mut Vector2<Real>) -> bool {
        // 3 points
        if simplex.len() == 3 {
            let a = simplex[2];
            let ao = a.neg();
            let b = simplex[1];
            let c = simplex[0];
            let ab = b - a;
            let ac = c - a;
            let ab_perp = ::util::triple_product(&ac, &ab, &ab);
            if ab_perp.dot(ao) > 0. {
                simplex.remove(0);
                *d = ab_perp;
            } else {
                let ac_perp = ::util::triple_product(&ab, &ac, &ac);
                if ac_perp.dot(ao) > 0. {
                    simplex.remove(1);
                    *d = ac_perp;
                } else {
                    return true;
                }
            }
        }
            // 2 points
            else if simplex.len() == 2 {
                let a = simplex[1];
                let ao = a.neg();
                let b = simplex[0];
                let ab = b - a;
                *d = ::util::triple_product(&ab, &ao, &ab);
            }
        // 0-1 point means we can't really do anything
        false
    }

    fn closest_feature(&self, simplex: &Vec<Vector2<Real>>) -> Option<Feature<Vector2<Real>>> {
        if simplex.len() < 3 {
            None
        } else {
            let mut feature = Feature::new();
            for i in 0..simplex.len() {
                let j = if i + 1 == simplex.len() { 0 } else { i + 1 };
                let a = simplex[i];
                let b = simplex[j];
                let e = b - a;
                let oa = a;
                let n = ::util::triple_product(&e, &oa, &e).normalize();
                let d = n.dot(a);
                if d < feature.distance {
                    feature.distance = d;
                    feature.normal = n;
                    feature.index = j;
                }
            }
            Some(feature)
        }
    }

    fn new() -> Self {
        Self {}
    }
}

#[cfg(test)]
mod tests {
    use cgmath::Vector2;

    use super::*;

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
        let mut simplex = vec![Vector2::new(40., 0.)];
        assert!(!processor.check_origin(&mut simplex, &mut direction));
        assert_eq!(1, simplex.len());
        assert_eq!(Vector2::new(1., 0.), direction);
    }

    #[test]
    fn test_check_origin_edge() {
        let processor = SimplexProcessor2D::new();
        let mut direction = Vector2::new(1., 0.);
        let mut simplex = vec![Vector2::new(40., 10.), Vector2::new(-10., 10.)];
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
            Vector2::new(40., 10.),
            Vector2::new(-10., 10.),
            Vector2::new(0., 3.),
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
            Vector2::new(40., 10.),
            Vector2::new(10., 10.),
            Vector2::new(3., -3.),
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
            Vector2::new(40., 10.),
            Vector2::new(-10., 10.),
            Vector2::new(0., -3.),
        ];
        assert!(processor.check_origin(&mut simplex, &mut direction));
        assert_eq!(3, simplex.len());
        assert_eq!(Vector2::new(1., 0.), direction);
    }

    #[test]
    fn test_closest_feature_0() {
        let processor = SimplexProcessor2D::new();
        assert!(processor.closest_feature(&vec![]).is_none())
    }

    #[test]
    fn test_closest_feature_1() {
        let processor = SimplexProcessor2D::new();
        assert!(
        processor
            .closest_feature(&vec![Vector2::new(10., 10.)])
            .is_none()
        )
    }

    #[test]
    fn test_closest_feature_2() {
        let processor = SimplexProcessor2D::new();
        assert!(
        processor
            .closest_feature(&vec![Vector2::new(10., 10.), Vector2::new(-10., 5.)])
            .is_none()
        )
    }

    #[test]
    fn test_closest_feature_3() {
        let processor = SimplexProcessor2D::new();
        let feature = processor.closest_feature(&vec![
            Vector2::new(10., 10.),
            Vector2::new(-10., 5.),
            Vector2::new(2., -5.),
        ]);
        assert!(feature.is_some());
        let feature = feature.unwrap();
        assert_eq!(2, feature.index);
        assert_approx_eq!(2.5607374, feature.distance);
        assert_approx_eq!(-0.6401844, feature.normal.x);
        assert_approx_eq!(-0.7682213, feature.normal.y);
    }
}
