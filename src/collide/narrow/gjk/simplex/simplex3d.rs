use std::ops::Neg;

use cgmath::{Point3, Vector3};
use cgmath::prelude::*;

use super::SimplexProcessor;
use Real;
use collide::narrow::gjk::SupportPoint;

/// Simplex processor implementation for 3D. Only to be used in [`GJK`](struct.GJK.html).
#[derive(Debug)]
pub struct SimplexProcessor3;

impl SimplexProcessor for SimplexProcessor3 {
    type Vector = Vector3<Real>;
    type Point = Point3<Real>;

    fn check_origin(
        &self,
        simplex: &mut Vec<SupportPoint<Point3<Real>>>,
        v: &mut Vector3<Real>,
    ) -> bool {
        // 4 points
        if simplex.len() == 4 {
            let a = simplex[3].v;
            let b = simplex[2].v;
            let c = simplex[1].v;
            let d = simplex[0].v;

            let ao = a.neg();
            let ab = b - a;
            let ac = c - a;

            let abc = ab.cross(ac);

            // origin outside ABC, remove D and check side
            // need to check both edges
            if abc.dot(ao) > 0. {
                simplex.remove(0);
                check_side(&abc, &ab, &ac, &ao, simplex, v, true, false);
            } else {
                let ad = d - a;
                let acd = ac.cross(ad);
                // origin outside ACD, remove B and check side
                // no need to test first edge, since that region is also over ABC
                if acd.dot(ao) > 0. {
                    simplex.remove(2);
                    check_side(&acd, &ac, &ad, &ao, simplex, v, true, true);
                } else {
                    let adb = ad.cross(ab);
                    // origin outside ADB, remove C and check side
                    // no need to test edges, since those regions are covered in earlier tests
                    if adb.dot(ao) > 0. {
                        // [b, d, a]
                        simplex.remove(1);
                        simplex.swap(0, 1);
                        *v = adb;
                    // origin is inside simplex
                    } else {
                        return true;
                    }
                }
            }
        }
        // 3 points
        else if simplex.len() == 3 {
            let a = simplex[2].v;
            let b = simplex[1].v;
            let c = simplex[0].v;

            let ao = a.neg();
            let ab = b - a;
            let ac = c - a;

            check_side(&ab.cross(ac), &ab, &ac, &ao, simplex, v, false, false);
        }
        // 2 points
        else if simplex.len() == 2 {
            let a = simplex[1].v;
            let b = simplex[0].v;

            let ao = a.neg();
            let ab = b - a;

            *v = cross_aba(&ab, &ao);
        }
        // 0-1 points
        false
    }

    fn new() -> Self {
        Self {}
    }
}

#[inline]
fn cross_aba(a: &Vector3<Real>, b: &Vector3<Real>) -> Vector3<Real> {
    a.cross(*b).cross(*a)
}

#[inline]
fn check_side(
    abc: &Vector3<Real>,
    ab: &Vector3<Real>,
    ac: &Vector3<Real>,
    ao: &Vector3<Real>,
    simplex: &mut Vec<SupportPoint<Point3<Real>>>,
    v: &mut Vector3<Real>,
    above: bool,
    ignore_ab: bool,
) {
    let ab_perp = ab.cross(*abc);

    // origin outside AB, remove C and v = edge normal towards origin
    if !ignore_ab && ab_perp.dot(*ao) > 0. {
        simplex.remove(0);
        *v = cross_aba(ab, ao);
        return;
    }

    let ac_perp = abc.cross(*ac);

    // origin outside AC, remove B and v = edge normal towards origin
    if ac_perp.dot(*ao) > 0. {
        simplex.remove(1);
        *v = cross_aba(ac, ao);
        return;
        // origin above triangle, set v = surface normal towards origin
    }

    if above {
        *v = *abc;
    } else if abc.dot(*ao) > 0. {
        // [c, b, a]
        *v = *abc;
    // origin below triangle, rewind simplex and set v = surface normal towards origin
    } else {
        // [b, c, a]
        simplex.swap(0, 1);
        *v = abc.neg();
    }
}

#[cfg(test)]
mod tests {
    use std::ops::Neg;

    use cgmath::{Point3, Vector3};

    use super::*;
    use collide::narrow::gjk::SupportPoint;

    #[test]
    fn test_check_side_outside_ab() {
        let mut simplex = vec![sup(8., -10., 0.), sup(-1., -10., 0.), sup(3., 5., 0.)];
        let v = test_check_side(&mut simplex, false, false);
        assert_eq!(2, simplex.len());
        assert_eq!(Vector3::new(-1., -10., 0.), simplex[0].v); // B should be last in the simplex
        assert_approx_eq!(-375., v.x);
        assert_approx_eq!(100., v.y);
        assert_approx_eq!(0., v.z);
    }

    #[test]
    fn test_check_side_outside_ac() {
        let mut simplex = vec![sup(2., -10., 0.), sup(-7., -10., 0.), sup(-3., 5., 0.)];
        let v = test_check_side(&mut simplex, false, false);
        assert_eq!(2, simplex.len());
        assert_eq!(Vector3::new(2., -10., 0.), simplex[0].v); // C should be last in the simplex
        assert_approx_eq!(300., v.x);
        assert_approx_eq!(100., v.y);
        assert_approx_eq!(0., v.z);
    }

    #[test]
    fn test_check_side_above() {
        let mut simplex = vec![sup(5., -10., -1.), sup(-4., -10., -1.), sup(0., 5., -1.)];
        let v = test_check_side(&mut simplex, false, false);
        assert_eq!(3, simplex.len());
        assert_eq!(Vector3::new(5., -10., -1.), simplex[0].v); // C should be last in the simplex
        assert_approx_eq!(0., v.x);
        assert_approx_eq!(0., v.y);
        assert_approx_eq!(135., v.z);
    }

    #[test]
    fn test_check_side_below() {
        let mut simplex = vec![sup(5., -10., 1.), sup(-4., -10., 1.), sup(0., 5., 1.)];
        let v = test_check_side(&mut simplex, false, false);
        assert_eq!(3, simplex.len());
        assert_eq!(Vector3::new(-4., -10., 1.), simplex[0].v); // B should be last in the simplex
        assert_approx_eq!(0., v.x);
        assert_approx_eq!(0., v.y);
        assert_approx_eq!(-135., v.z);
    }

    #[test]
    fn test_check_origin_empty() {
        let mut simplex = vec![];
        let (hit, v) = test_check_origin(&mut simplex);
        assert!(!hit);
        assert!(simplex.is_empty());
        assert_eq!(Vector3::zero(), v);
    }

    #[test]
    fn test_check_origin_point() {
        let mut simplex = vec![sup(8., -10., 0.)];
        let (hit, v) = test_check_origin(&mut simplex);
        assert!(!hit);
        assert_eq!(1, simplex.len());
        assert_eq!(Vector3::zero(), v);
    }

    #[test]
    fn test_check_origin_line() {
        let mut simplex = vec![sup(8., -10., 0.), sup(-1., -10., 0.)];
        let (hit, v) = test_check_origin(&mut simplex);
        assert!(!hit);
        assert_eq!(2, simplex.len());
        assert_eq!(Vector3::new(0., 810., 0.), v);
    }

    #[test]
    fn test_check_origin_triangle() {
        let mut simplex = vec![sup(5., -10., -1.), sup(-4., -10., -1.), sup(0., 5., -1.)];
        let (hit, v) = test_check_origin(&mut simplex);
        assert!(!hit);
        assert_eq!(3, simplex.len());
        assert_eq!(Vector3::new(0., 0., 135.), v);
    }

    #[test]
    fn test_check_origin_tetrahedron_outside_abc() {
        let mut simplex = vec![
            sup(8., -10., -1.),
            sup(-1., -10., -1.),
            sup(3., 5., -1.),
            sup(3., -3., 5.),
        ];
        let (hit, v) = test_check_origin(&mut simplex);
        assert!(!hit);
        assert_eq!(3, simplex.len());
        assert_eq!(Vector3::new(-1., -10., -1.), simplex[0].v);
        assert_eq!(Vector3::new(-90., 24., 32.), v);
    }

    #[test]
    fn test_check_origin_tetrahedron_outside_acd() {
        let mut simplex = vec![
            sup(8., 0.1, -1.),
            sup(-1., 0.1, -1.),
            sup(3., 15., -1.),
            sup(3., 7., 5.),
        ];
        let (hit, v) = test_check_origin(&mut simplex);
        assert!(!hit);
        assert_eq!(3, simplex.len());
        assert_eq!(Vector3::new(3., 7., 5.), simplex[2].v);
        assert_eq!(Vector3::new(-1., 0.1, -1.), simplex[1].v);
        assert_eq!(Vector3::new(8., 0.1, -1.), simplex[0].v);
        assert_eq!(Vector3::new(0., -54., 62.1), v);
    }

    #[test]
    fn test_check_origin_tetrahedron_outside_adb() {
        let mut simplex = vec![
            sup(2., -10., -1.),
            sup(-7., -10., -1.),
            sup(-3., 5., -1.),
            sup(-3., -3., 5.),
        ];
        let (hit, v) = test_check_origin(&mut simplex);
        assert!(!hit);
        assert_eq!(3, simplex.len());
        assert_eq!(Vector3::new(-3., -3., 5.), simplex[2].v);
        assert_eq!(Vector3::new(2., -10., -1.), simplex[1].v);
        assert_eq!(Vector3::new(-3., 5., -1.), simplex[0].v);
        assert_eq!(Vector3::new(90., 30., 40.), v);
    }

    #[test]
    fn test_check_origin_tetrahedron_inside() {
        let mut simplex = vec![
            sup(3., -3., -1.),
            sup(-3., -3., -1.),
            sup(0., 3., -1.),
            sup(0., 0., 5.),
        ];
        let (hit, _) = test_check_origin(&mut simplex);
        assert!(hit);
        assert_eq!(4, simplex.len());
    }

    fn test_check_origin(simplex: &mut Vec<SupportPoint<Point3<Real>>>) -> (bool, Vector3<Real>) {
        let mut v = Vector3::zero();
        let b = SimplexProcessor3.check_origin(simplex, &mut v);
        (b, v)
    }

    fn test_check_side(
        simplex: &mut Vec<SupportPoint<Point3<Real>>>,
        above: bool,
        ignore: bool,
    ) -> Vector3<Real> {
        let ab = simplex[1].v - simplex[2].v;
        let ac = simplex[0].v - simplex[2].v;
        let ao = simplex[2].v.neg();
        let abc = ab.cross(ac);
        let mut v = Vector3::zero();
        check_side(&abc, &ab, &ac, &ao, simplex, &mut v, above, ignore);
        v
    }

    fn sup(x: Real, y: Real, z: Real) -> SupportPoint<Point3<Real>> {
        let mut s = SupportPoint::new();
        s.v = Vector3::new(x, y, z);
        s
    }
}
