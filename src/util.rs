use cgmath::prelude::*;
use cgmath::{BaseFloat, Vector2, Decomposed};
use collision::{Aabb, MinMax};

pub fn get_max_point<S, V, P, R>(vertices: &Vec<V>, direction: &V, transform: &Decomposed<V, R>) -> P
where
    S: BaseFloat,
    V: VectorSpace<Scalar = S> + ElementWise + Array<Element = S> + InnerSpace,
    P: EuclideanSpace<Scalar = S, Diff = V>,
    R: Rotation<P>,
{
    let direction = transform.rot.invert().rotate_vector(*direction);
    let (p, _) = vertices
        .iter()
        .map(|v| (v, v.dot(direction)))
        .fold((V::zero(), S::neg_infinity()), |(max_p, max_dot),
         (v, dot)| {
            if dot > max_dot {
                (v.clone(), dot)
            } else {
                (max_p, max_dot)
            }
        });
    P::from_vec(transform.transform_vector(p))
}

pub fn get_bound<S, V, P, A>(vertices: &Vec<V>) -> A
where
    S: BaseFloat,
    V: VectorSpace<Scalar=S> + ElementWise + Array<Element=S>,
    P: EuclideanSpace<Scalar=S, Diff=V> + MinMax,
    A: Aabb<S, V, P>,
{
    vertices.iter().fold(
        A::new(P::from_value(S::zero()), P::from_value(S::zero())),
        |bound, p| bound.grow(P::from_vec(*p))
    )
}

#[inline]
pub fn triple_product<S: BaseFloat>(a: &Vector2<S>, b: &Vector2<S>, c: &Vector2<S>) -> Vector2<S> {
    let ac = a.x * c.x + a.y * c.y;
    let bc = b.x * c.x + b.y * c.y;
    Vector2::new(b.x * ac - a.x * bc, b.y * ac - a.y * bc)
}
