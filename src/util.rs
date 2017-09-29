use cgmath::Vector2;
use cgmath::prelude::*;

use Real;

#[inline]
pub(crate) fn triple_product(
    a: &Vector2<Real>,
    b: &Vector2<Real>,
    c: &Vector2<Real>,
) -> Vector2<Real> {
    let ac = a.x * c.x + a.y * c.y;
    let bc = b.x * c.x + b.y * c.y;
    Vector2::new(b.x * ac - a.x * bc, b.y * ac - a.y * bc)
}

pub(crate) fn barycentric_vector<V>(p: V, a: V, b: V, c: V) -> (Real, Real, Real)
where
    V: VectorSpace<Scalar = Real> + InnerSpace,
{
    let v0 = b - a;
    let v1 = c - a;
    let v2 = p - a;
    let d00 = v0.dot(v0);
    let d01 = v0.dot(v1);
    let d11 = v1.dot(v1);
    let d20 = v2.dot(v0);
    let d21 = v2.dot(v1);
    let inv_denom = 1. / (d00 * d11 - d01 * d01);

    let v = (d11 * d20 - d01 * d21) * inv_denom;
    let w = (d00 * d21 - d01 * d20) * inv_denom;
    let u = 1. - v - w;
    (u, v, w)
}
