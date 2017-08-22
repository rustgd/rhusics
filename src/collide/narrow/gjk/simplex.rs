use cgmath::prelude::*;
use cgmath::{BaseFloat, Vector2, Vector3};
use std::ops::Neg;

pub trait SimplexProcessor<S, V>
where
    S: BaseFloat,
    V: VectorSpace<Scalar = S>
        + ElementWise
        + Array<Element = S>
        + InnerSpace,
{
    fn check_origin(&self, simplex: &mut Vec<V>, d: &mut V) -> bool;
    fn closest_feature(&self, simplex: &Vec<V>) -> Option<Feature<S, V>>;
    fn new() -> Self;
}

#[derive(Debug)]
pub struct Feature<S, V>
where
    S: BaseFloat,
    V: VectorSpace<Scalar = S> + ElementWise + Array<Element = S> + InnerSpace,
{
    pub normal: V,
    pub distance: S,
    pub index: usize,
}

impl<S, V> Feature<S, V>
where
    S: BaseFloat,
    V: VectorSpace<Scalar = S> + ElementWise + Array<Element = S> + InnerSpace,
{
    pub fn new() -> Self {
        Self {
            normal: V::zero(),
            distance: S::infinity(),
            index: 0,
        }
    }
}

pub struct SimplexProcessor2D;
pub struct SimplexProcessor3D;

impl<S> SimplexProcessor<S, Vector2<S>> for SimplexProcessor2D
where
    S: BaseFloat,
{
    fn check_origin(&self, simplex: &mut Vec<Vector2<S>>, d: &mut Vector2<S>) -> bool {
        // 3 points
        if simplex.len() == 3 {
            let a = simplex[2];
            let ao = a.neg();
            let b = simplex[1];
            let c = simplex[0];
            let ab = b - a;
            let ac = c - a;
            let ab_perp = ::util::triple_product(&ac, &ab, &ab);
            if ab_perp.dot(ao) > S::zero() {
                simplex.remove(0);
                *d = ab_perp;
            } else {
                let ac_perp = ::util::triple_product(&ab, &ac, &ac);
                if ac_perp.dot(ao) > S::zero() {
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

    fn closest_feature(&self, simplex: &Vec<Vector2<S>>) -> Option<Feature<S, Vector2<S>>> {
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

impl<S> SimplexProcessor<S, Vector3<S>> for SimplexProcessor3D
where
    S: BaseFloat,
{
    fn check_origin(&self, simplex: &mut Vec<Vector3<S>>, d: &mut Vector3<S>) -> bool {
        // 4 points
        if simplex.len() == 4 {
            //TODO
        }
        // 3 points
        else if simplex.len() == 3 {
            //TODO
        }
        // 2 points
        else if simplex.len() == 2 {
            let a = simplex[1];
            let ao = a.neg();
            let b = simplex[0];
            let ab = b - a;
            if ab.dot(ao) > S::zero() {
                *d = ab.cross(ao).cross(ab);
            } else {
                simplex.remove(0);
                *d = ao;
            }
        }
        // 0-1 points
        false
    }

    fn closest_feature(&self, simplex: &Vec<Vector3<S>>) -> Option<Feature<S, Vector3<S>>> {
        if simplex.len() < 4 {
            None
        } else {
            None // TODO
        }
    }

    fn new() -> Self {
        Self {}
    }
}

#[cfg(test)]
mod tests_2d {
    use super::*;

    use cgmath::Vector2;

    #[test]
    fn test_check_origin_empty() {
        let processor =
            <SimplexProcessor2D as SimplexProcessor<f32, Vector2<f32>>>::new();
        let mut direction = Vector2::new(1., 0.);
        let mut simplex = vec![];
        assert!(!processor.check_origin(&mut simplex, &mut direction));
        assert_eq!(0, simplex.len());
        assert_eq!(Vector2::new(1., 0.), direction);
    }

    #[test]
    fn test_check_origin_single() {
        let processor =
            <SimplexProcessor2D as SimplexProcessor<f32, Vector2<f32>>>::new();
        let mut direction = Vector2::new(1., 0.);
        let mut simplex = vec![Vector2::new(40., 0.)];
        assert!(!processor.check_origin(&mut simplex, &mut direction));
        assert_eq!(1, simplex.len());
        assert_eq!(Vector2::new(1., 0.), direction);
    }

    #[test]
    fn test_check_origin_edge() {
        let processor =
            <SimplexProcessor2D as SimplexProcessor<f32, Vector2<f32>>>::new();
        let mut direction = Vector2::new(1., 0.);
        let mut simplex = vec![Vector2::new(40., 10.), Vector2::new(-10., 10.)];
        assert!(!processor.check_origin(&mut simplex, &mut direction));
        assert_eq!(2, simplex.len());
        assert_eq!(0., direction.x);
        assert!(direction.y < 0.);
    }

    #[test]
    fn test_check_origin_triangle_outside_ac() {
        let processor =
            <SimplexProcessor2D as SimplexProcessor<f32, Vector2<f32>>>::new();
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
        let processor =
            <SimplexProcessor2D as SimplexProcessor<f32, Vector2<f32>>>::new();
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
        let processor =
            <SimplexProcessor2D as SimplexProcessor<f32, Vector2<f32>>>::new();
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
        let processor =
            <SimplexProcessor2D as SimplexProcessor<f32, Vector2<f32>>>::new();
        assert!(
            <SimplexProcessor2D as SimplexProcessor<f32, Vector2<f32>>>::closest_feature(
                &processor,
                &vec![],
            ).is_none()
        )
    }

    #[test]
    fn test_closest_feature_1() {
        let processor =
            <SimplexProcessor2D as SimplexProcessor<f32, Vector2<f32>>>::new();
        assert!(
            processor
                .closest_feature(&vec![Vector2::<f32>::new(10., 10.)])
                .is_none()
        )
    }

    #[test]
    fn test_closest_feature_2() {
        let processor =
            <SimplexProcessor2D as SimplexProcessor<f32, Vector2<f32>>>::new();
        assert!(
            processor
                .closest_feature(&vec![Vector2::<f32>::new(10., 10.),
                                       Vector2::<f32>::new(-10., 5.)])
                .is_none()
        )
    }

    #[test]
    fn test_closest_feature_3() {
        let processor =
            <SimplexProcessor2D as SimplexProcessor<f32, Vector2<f32>>>::new();
        let feature = processor
            .closest_feature(&vec![Vector2::<f32>::new(10., 10.),
                                   Vector2::<f32>::new(-10., 5.),
                                   Vector2::<f32>::new(2., -5.)]);
        assert!(feature.is_some());
        let feature = feature.unwrap();
        assert_eq!(2, feature.index);
        assert_eq!(2.5607374, feature.distance);
        assert_eq!(Vector2::new(-0.6401844, -0.7682213), feature.normal);
    }
}
