use std::ops::Neg;

use cgmath::prelude::*;
use cgmath::{Vector3};

use Real;
use super::{SimplexProcessor, Feature};

pub struct SimplexProcessor3D;

impl SimplexProcessor for SimplexProcessor3D {
    type Vector = Vector3<Real>;

    fn check_origin(&self, simplex: &mut Vec<Vector3<Real>>, v: &mut Vector3<Real>) -> bool {
        // 4 points
        if simplex.len() == 4 {
            let a = simplex[3];
            let b = simplex[2];
            let c = simplex[1];
            let d = simplex[0];

            let ao = a.neg();
            let ab = b - a;
            let ac = c - a;
            let ad = d - a;

            let abc = ab.cross(ac);

            // origin outside ABC, remove D and check side
            // need to check both edges
            if abc.dot(ao) > 0. {
                simplex.remove(0);
                check_side(&abc, &ab, &ac, &ao, simplex, v, true, false);
            } else {
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
                let a = simplex[2];
                let b = simplex[1];
                let c = simplex[0];
                let ao = a.neg();
                let ab = b - a;
                let ac = c - a;

                check_side(&ab.cross(ac), &ab, &ac, &ao, simplex, v, false, false);
            }
                // 2 points
                else if simplex.len() == 2 {
                    let a = simplex[1];
                    let ao = a.neg();
                    let b = simplex[0];
                    let ab = b - a;
                    *v = cross_aba(&ab, &ao);
                }
        // 0-1 points
        false
    }

    fn closest_feature(&self, simplex: &Vec<Vector3<Real>>) -> Option<Feature<Vector3<Real>>> {
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

#[inline]
fn cross_aba(a: &Vector3<Real>, b: &Vector3<Real>) -> Vector3<Real> {
    a.cross(*b).cross(*a)
}

#[inline]
fn check_side(abc: &Vector3<Real>, ab: &Vector3<Real>, ac: &Vector3<Real>, ao: &Vector3<Real>,
              simplex: &mut Vec<Vector3<Real>>, v : &mut Vector3<Real>, above : bool, ignore_ab : bool) {
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

}