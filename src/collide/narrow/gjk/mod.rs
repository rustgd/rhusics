use std::fmt::Debug;
use std::ops::Neg;

use cgmath::prelude::*;
use collision::prelude::*;

use self::epa::{EPA2, EPA3, EPA};
use self::simplex::{SimplexProcessor, SimplexProcessor2, SimplexProcessor3};
use super::NarrowPhase;
use Real;
use collide::{CollisionShape, CollisionStrategy, Contact, Primitive};

mod simplex;
mod epa;

const MAX_ITERATIONS: u32 = 100;

/// GJK algorithm for 2D, see [GJK](struct.GJK.html) for more information.
pub type GJK2 = GJK<SimplexProcessor2, EPA2>;

/// GJK algorithm for 3D, see [GJK](struct.GJK.html) for more information.
pub type GJK3 = GJK<SimplexProcessor3, EPA3>;

/// Gilbert-Johnson-Keerthi narrow phase collision detection algorithm.
///
/// # Type parameters:
///
/// - `S`: simplex processor type. Should be either
///        [`SimplexProcessor2`](struct.SimplexProcessor2.html) or
///        [`SimplexProcessor3`](struct.SimplexProcessor3.html)
/// - `E`: EPA algorithm implementation type. Should be either
///        [`EPA2`](struct.EPA2.html) or
///        [`EPA3`](struct.EPA3.html)
///
#[derive(Debug)]
pub struct GJK<SP, E> {
    simplex_processor: SP,
    epa: E,
}

impl<SP, E> GJK<SP, E>
where
    SP: SimplexProcessor,
    E: EPA<Point = SP::Point>,
{
    /// Create a new GJK algorithm implementation
    pub fn new() -> Self {
        Self {
            simplex_processor: SP::new(),
            epa: E::new(),
        }
    }

    /// Do intersection test on the given primitives
    pub fn intersect<P, PL, PR, TL, TR>(
        &self,
        left: &PL,
        left_transform: &TL,
        right: &PR,
        right_transform: &TR,
    ) -> Option<Vec<SupportPoint<P>>>
    where
        P: EuclideanSpace<Scalar = Real>,
        PL: SupportFunction<Point = P>,
        PR: SupportFunction<Point = P>,
        SP: SimplexProcessor<Point = P>,
        P::Diff: Neg<Output = P::Diff> + InnerSpace,
        TL: Transform<P>,
        TR: Transform<P>,
    {
        let right_pos = right_transform.transform_point(P::from_value(0.));
        let left_pos = left_transform.transform_point(P::from_value(0.));
        let mut d = right_pos - left_pos;
        let a = SupportPoint::from_minkowski(left, left_transform, right, right_transform, &d);
        if a.v.dot(d) <= 0. {
            return None;
        }
        let mut simplex: Vec<SupportPoint<P>> = Vec::default();
        simplex.push(a);
        d = d.neg();
        let mut i = 0;
        loop {
            let a = SupportPoint::from_minkowski(left, left_transform, right, right_transform, &d);
            if a.v.dot(d) <= 0. {
                return None;
            } else {
                simplex.push(a);
                if self.simplex_processor.check_origin(&mut simplex, &mut d) {
                    return Some(simplex);
                }
            }
            i += 1;
            if i >= MAX_ITERATIONS {
                return None;
            }
        }
    }

    /// Given a simplex that encloses the origin, compute the closest point on the Minkowski sum,
    /// to the origin.
    pub fn get_contact_manifold<P, PL, PR, TL, TR>(
        &self,
        mut simplex: &mut Vec<SupportPoint<P>>,
        left: &PL,
        left_transform: &TL,
        right: &PR,
        right_transform: &TR,
    ) -> Option<Contact<P>>
    where
        P: EuclideanSpace<Scalar = Real>,
        PL: SupportFunction<Point = P>,
        PR: SupportFunction<Point = P>,
        TL: Transform<P>,
        TR: Transform<P>,
        SP: SimplexProcessor<Point = P>,
    {
        self.epa
            .process(&mut simplex, left, left_transform, right, right_transform)
    }

    /// Do intersection test on the given primitives, and return the actual intersection point
    pub fn intersection<P, PL, PR, TL, TR>(
        &self,
        strategy: &CollisionStrategy,
        left: &PL,
        left_transform: &TL,
        right: &PR,
        right_transform: &TR,
    ) -> Option<Contact<P>>
    where
        P: EuclideanSpace<Scalar = Real>,
        P::Diff: Neg<Output = P::Diff> + InnerSpace,
        PL: SupportFunction<Point = P>,
        PR: SupportFunction<Point = P>,
        TL: Transform<P>,
        TR: Transform<P>,
        SP: SimplexProcessor<Point = P>,
    {
        match self.intersect(left, left_transform, right, right_transform) {
            None => None,
            Some(mut simplex) => match *strategy {
                CollisionStrategy::CollisionOnly => {
                    Some(Contact::new(CollisionStrategy::CollisionOnly))
                }
                CollisionStrategy::FullResolution => self.get_contact_manifold(
                    &mut simplex,
                    left,
                    left_transform,
                    right,
                    right_transform,
                ),
            },
        }
    }
}

impl<P, T, S, E> NarrowPhase<P, T> for GJK<S, E>
where
    P: Primitive,
    P::Aabb: Discrete<P::Aabb> + Aabb<Scalar = Real>,
    P::Point: Debug,
    <P::Point as EuclideanSpace>::Diff: InnerSpace
        + Neg<Output = <P::Point as EuclideanSpace>::Diff>
        + Debug,
    S: SimplexProcessor<Point = P::Point> + Debug,
    T: Transform<P::Point> + Debug,
    E: EPA<Point = P::Point> + Debug,
{
    fn collide(
        &self,
        left: &CollisionShape<P, T>,
        left_transform: &T,
        right: &CollisionShape<P, T>,
        right_transform: &T,
    ) -> Option<Contact<P::Point>> {
        if !left.enabled || !right.enabled || left.primitives.is_empty()
            || right.primitives.is_empty()
        {
            return None;
        }

        let strategy = max(&left.strategy, &right.strategy);
        let mut contacts = Vec::default();
        for &(ref left_primitive, ref left_local_transform) in &left.primitives {
            let left_transform = left_transform.concat(left_local_transform);
            for &(ref right_primitive, ref right_local_transform) in &right.primitives {
                let right_transform = right_transform.concat(right_local_transform);
                match self.intersection(
                    &strategy,
                    left_primitive,
                    &left_transform,
                    right_primitive,
                    &right_transform,
                ) {
                    Some(contact) => contacts.push(contact),
                    None => (),
                };
            }
        }

        if contacts.len() > 0 {
            match strategy {
                CollisionStrategy::CollisionOnly => Some(contacts[0].clone()),
                CollisionStrategy::FullResolution => {
                    // penetration depth defaults to 0., and can't be nan from epa,
                    // so unwrapping is safe
                    contacts
                        .iter()
                        .max_by(|l, r| {
                            l.penetration_depth
                                .partial_cmp(&r.penetration_depth)
                                .unwrap()
                        })
                        .cloned()
                }
            }
        } else {
            None
        }
    }
}

fn max(left: &CollisionStrategy, right: &CollisionStrategy) -> CollisionStrategy {
    if left > right {
        left.clone()
    } else {
        right.clone()
    }
}

#[derive(Clone, Debug)]
pub struct SupportPoint<P>
where
    P: EuclideanSpace,
{
    v: P::Diff,
    sup_a: P,
    sup_b: P,
}

impl<P> SupportPoint<P>
where
    P: EuclideanSpace<Scalar = Real>,
{
    pub fn new() -> Self {
        Self {
            v: P::Diff::zero(),
            sup_a: P::from_value(0.),
            sup_b: P::from_value(0.),
        }
    }

    pub fn from_minkowski<SL, SR, TL, TR>(
        left: &SL,
        left_transform: &TL,
        right: &SR,
        right_transform: &TR,
        direction: &P::Diff,
    ) -> Self
    where
        SL: SupportFunction<Point = P>,
        SR: SupportFunction<Point = P>,
        P::Diff: Neg<Output = P::Diff>,
        TL: Transform<P>,
        TR: Transform<P>,
    {
        let l = left.support_point(direction, left_transform);
        let r = right.support_point(&direction.neg(), right_transform);
        Self {
            v: l - r,
            sup_a: l,
            sup_b: r,
        }
    }
}

#[cfg(test)]
mod tests {
    use cgmath::{Point2, Point3, Quaternion, Rad, Rotation2, Rotation3, Vector2};

    use super::SupportPoint;
    use Real;
    use collide::narrow::NarrowPhase;
    use collide2d::*;
    use collide3d::*;

    fn transform(x: Real, y: Real, angle: Real) -> BodyPose2 {
        BodyPose2::new(Point2::new(x, y), Rotation2::from_angle(Rad(angle)))
    }

    fn transform_3d(x: Real, y: Real, z: Real, angle_z: Real) -> BodyPose3 {
        BodyPose3::new(Point3::new(x, y, z), Quaternion::from_angle_z(Rad(angle_z)))
    }

    #[test]
    fn test_support() {
        let left = Rectangle::new(10., 10.);
        let left_transform = transform(15., 0., 0.);
        let right = Rectangle::new(10., 10.);
        let right_transform = transform(-15., 0., 0.);
        let direction = Vector2::new(1., 0.);
        assert_eq!(
            Vector2::new(40., 0.),
            SupportPoint::from_minkowski(
                &left,
                &left_transform,
                &right,
                &right_transform,
                &direction,
            ).v
        );
    }

    #[test]
    fn test_gjk_miss() {
        let left = Rectangle::new(10., 10.);
        let left_transform = transform(15., 0., 0.);
        let right = Rectangle::new(10., 10.);
        let right_transform = transform(-15., 0., 0.);
        let gjk = GJK2::new();
        assert!(
            gjk.intersect(&left, &left_transform, &right, &right_transform)
                .is_none()
        );
    }

    #[test]
    fn test_gjk_hit() {
        let left = Rectangle::new(10., 10.);
        let left_transform = transform(15., 0., 0.);
        let right = Rectangle::new(10., 10.);
        let right_transform = transform(7., 2., 0.);
        let gjk = GJK2::new();
        let simplex = gjk.intersect(&left, &left_transform, &right, &right_transform);
        assert!(simplex.is_some());
    }

    #[test]
    fn test_gjk_3d_hit() {
        let left = Cuboid::new(10., 10., 10.);
        let left_transform = transform_3d(15., 0., 0., 0.);
        let right = Cuboid::new(10., 10., 10.);
        let right_transform = transform_3d(7., 2., 0., 0.);
        let gjk = GJK3::new();
        let simplex = gjk.intersect(&left, &left_transform, &right, &right_transform);
        assert!(simplex.is_some());
    }

    #[test]
    fn test_gjk_shape_simple_miss() {
        let left = CollisionShape2::new_simple(
            CollisionStrategy::CollisionOnly,
            Rectangle::new(10., 10.).into(),
        );
        let left_transform = transform(15., 0., 0.);
        let right = CollisionShape2::new_simple(
            CollisionStrategy::CollisionOnly,
            Rectangle::new(10., 10.).into(),
        );
        let right_transform = transform(-15., 0., 0.);
        let gjk = GJK2::new();
        assert!(
            gjk.collide(&left, &left_transform, &right, &right_transform)
                .is_none()
        );
    }

    #[test]
    fn test_gjk_shape_simple_hit() {
        let left = CollisionShape2::new_simple(
            CollisionStrategy::CollisionOnly,
            Rectangle::new(10., 10.).into(),
        );
        let left_transform = transform(15., 0., 0.);
        let right = CollisionShape2::new_simple(
            CollisionStrategy::CollisionOnly,
            Rectangle::new(10., 10.).into(),
        );
        let right_transform = transform(7., 2., 0.);
        let gjk = GJK2::new();
        let contact = gjk.collide(&left, &left_transform, &right, &right_transform);
        assert!(contact.is_some());
        let contact = contact.unwrap();
        assert_eq!(CollisionStrategy::CollisionOnly, contact.strategy);
    }

    #[test]
    fn test_gjk_3d_shape_hit() {
        let left = CollisionShape3::new_simple(
            CollisionStrategy::CollisionOnly,
            Cuboid::new(10., 10., 10.).into(),
        );
        let left_transform = transform_3d(15., 0., 0., 0.);
        let right = CollisionShape3::new_simple(
            CollisionStrategy::CollisionOnly,
            Cuboid::new(10., 10., 10.).into(),
        );
        let right_transform = transform_3d(7., 2., 0., 0.);
        let gjk = GJK3::new();
        let contact = gjk.collide(&left, &left_transform, &right, &right_transform);
        assert!(contact.is_some());
        let contact = contact.unwrap();
        assert_eq!(CollisionStrategy::CollisionOnly, contact.strategy);
    }

    #[test]
    fn test_gjk_shape_complex_miss() {
        let left = CollisionShape2::new_complex(
            CollisionStrategy::CollisionOnly,
            vec![
                (Rectangle::new(10., 10.).into(), transform(0., 5., 0.)),
                (Rectangle::new(10., 10.).into(), transform(0., -5., 0.)),
            ],
        );
        let left_transform = transform(15., 0., 0.);
        let right = CollisionShape2::new_simple(
            CollisionStrategy::CollisionOnly,
            Rectangle::new(10., 10.).into(),
        );
        let right_transform = transform(-15., 0., 0.);
        let gjk = GJK2::new();
        assert!(
            gjk.collide(&left, &left_transform, &right, &right_transform)
                .is_none()
        );
    }

    #[test]
    fn test_gjk_shape_complex_hit_single() {
        let left = CollisionShape2::new_complex(
            CollisionStrategy::CollisionOnly,
            vec![
                (Rectangle::new(10., 10.).into(), transform(0., 5., 0.)),
                (Rectangle::new(10., 10.).into(), transform(0., -5., 0.)),
            ],
        );
        let left_transform = transform(15., 0., 0.);
        let right = CollisionShape2::new_simple(
            CollisionStrategy::CollisionOnly,
            Rectangle::new(10., 10.).into(),
        );
        let right_transform = transform(7., 6., 0.);
        let gjk = GJK2::new();
        let contact = gjk.collide(&left, &left_transform, &right, &right_transform);
        assert!(contact.is_some());
        let contact = contact.unwrap();
        assert_eq!(CollisionStrategy::CollisionOnly, contact.strategy);
    }

    #[test]
    fn test_gjk_shape_complex_hit_both() {
        let left = CollisionShape2::new_complex(
            CollisionStrategy::CollisionOnly,
            vec![
                (Rectangle::new(10., 10.).into(), transform(0., 5., 0.)),
                (Rectangle::new(10., 10.).into(), transform(0., -5., 0.)),
            ],
        );
        let left_transform = transform(15., 0., 0.);
        let right = CollisionShape2::new_simple(
            CollisionStrategy::CollisionOnly,
            Rectangle::new(10., 10.).into(),
        );
        let right_transform = transform(7., 4., 0.);
        let gjk = GJK2::new();
        let contact = gjk.collide(&left, &left_transform, &right, &right_transform);
        assert!(contact.is_some());
        let contact = contact.unwrap();
        assert_eq!(CollisionStrategy::CollisionOnly, contact.strategy);
    }
}
