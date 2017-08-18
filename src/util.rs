use cgmath;
use cgmath::{BaseFloat, VectorSpace, ElementWise, Array, InnerSpace, EuclideanSpace, Transform,
             Vector2};

pub fn get_max_point<S, V, P, T>(vertices: &Vec<V>, direction: &V, transform: &T) -> P
where
    S: BaseFloat,
    V: VectorSpace<Scalar = S> + ElementWise + Array<Element = S> + InnerSpace,
    P: EuclideanSpace<Scalar = S, Diff = V>,
    T: Transform<P>,
{
    let (p, _) = vertices
        .iter()
        .map(|v| transform.transform_vector(*v))
        .map(|v| (v, cgmath::dot(v, *direction)))
        .fold((V::zero(), S::neg_infinity()), |(max_p, max_dot),
         (v, dot)| {
            if dot > max_dot {
                (v.clone(), dot)
            } else {
                (max_p, max_dot)
            }
        });
    P::from_vec(p)
}

#[inline]
pub fn triple_product<S: BaseFloat>(a: &Vector2<S>, b: &Vector2<S>, c: &Vector2<S>) -> Vector2<S> {
    let ac = a.x * c.x + a.y * c.y;
    let bc = b.x * c.x + b.y * c.y;
    Vector2::new(b.x * ac - a.x * bc, b.y * ac - a.y * bc)
}
