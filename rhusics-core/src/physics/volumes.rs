use cgmath::{
    BaseFloat, EuclideanSpace, InnerSpace, Matrix3, Point2, Point3, SquareMatrix, Transform,
    Vector3, Zero,
};
use collision::primitive::*;
use collision::{Aabb, Aabb2, Aabb3, Bound, ComputeBound, Primitive, Union};

use super::{Inertia, Mass, Material, PartialCrossProduct};
use collide::CollisionShape;

/// Describe a shape with volume
///
/// ### Type parameters:
///
/// - `I`: Inertia type, see `Inertia` for more information
pub trait Volume<S, I> {
    /// Compute the mass of the shape based on its material
    fn get_mass(&self, material: &Material) -> Mass<S, I>;
}

impl<S> Volume<S, S> for Circle<S>
where
    S: BaseFloat + Inertia,
{
    fn get_mass(&self, material: &Material) -> Mass<S, S> {
        use std::f64::consts::PI;
        let pi = S::from(PI).unwrap();
        let mass = pi * self.radius * self.radius * material.density();
        let inertia = mass * self.radius * self.radius / (S::one() + S::one());
        Mass::new_with_inertia(mass, inertia)
    }
}

impl<S> Volume<S, S> for Rectangle<S>
where
    S: BaseFloat + Inertia,
{
    fn get_mass(&self, material: &Material) -> Mass<S, S> {
        let b: Aabb2<S> = self.compute_bound();
        let mass = b.volume() * material.density();
        let inertia =
            mass * (b.dim().x * b.dim().x + b.dim().y * b.dim().y) / S::from(12.).unwrap();
        Mass::new_with_inertia(mass, inertia)
    }
}

impl<S> Volume<S, S> for Square<S>
where
    S: BaseFloat + Inertia,
{
    fn get_mass(&self, material: &Material) -> Mass<S, S> {
        let b: Aabb2<S> = self.compute_bound();
        let mass = b.volume() * material.density();
        let inertia =
            mass * (b.dim().x * b.dim().x + b.dim().y * b.dim().y) / S::from(12.).unwrap();
        Mass::new_with_inertia(mass, inertia)
    }
}

impl<S> Volume<S, S> for ConvexPolygon<S>
where
    S: BaseFloat + Inertia,
{
    fn get_mass(&self, material: &Material) -> Mass<S, S> {
        let mut area = S::zero();
        let mut denom = S::zero();
        for i in 0..self.vertices.len() {
            let j = if i == self.vertices.len() - 1 {
                0
            } else {
                i + 1
            };
            let p0 = self.vertices[i].to_vec();
            let p1 = self.vertices[j].to_vec();
            let a = p0.cross(&p1).abs();
            let b = p0.dot(p0) + p0.dot(p1) + p1.dot(p1);
            denom += a * b;
            area += a;
        }
        let mass = area * S::from(0.5).unwrap() * material.density();
        let inertia = mass / S::from(6.).unwrap() * denom / area;
        Mass::new_with_inertia(mass, inertia)
    }
}

impl<S> Volume<S, Matrix3<S>> for Sphere<S>
where
    S: BaseFloat,
{
    fn get_mass(&self, material: &Material) -> Mass<S, Matrix3<S>> {
        use std::f64::consts::PI;
        let pi = S::from(PI).unwrap();
        let mass = S::from(4. / 3.).unwrap()
            * pi
            * self.radius
            * self.radius
            * self.radius
            * material.density();
        let inertia = S::from(2. / 5.).unwrap() * mass * self.radius * self.radius;
        Mass::new_with_inertia(mass, Matrix3::from_value(inertia))
    }
}

impl<S> Volume<S, Matrix3<S>> for Cuboid<S>
where
    S: BaseFloat,
{
    fn get_mass(&self, material: &Material) -> Mass<S, Matrix3<S>> {
        let b: Aabb3<S> = self.compute_bound();
        let mass = b.volume() * material.density();
        let x2 = b.dim().x * b.dim().x;
        let y2 = b.dim().y * b.dim().y;
        let z2 = b.dim().z * b.dim().z;
        let mnorm = mass / S::from(12.).unwrap();
        let inertia = Matrix3::from_diagonal(Vector3::new(y2 + z2, x2 + z2, x2 + y2) * mnorm);
        Mass::new_with_inertia(mass, inertia)
    }
}

impl<S> Volume<S, Matrix3<S>> for Cube<S>
where
    S: BaseFloat,
{
    fn get_mass(&self, material: &Material) -> Mass<S, Matrix3<S>> {
        let b: Aabb3<S> = self.compute_bound();
        let mass = b.volume() * material.density();
        let x2 = b.dim().x * b.dim().x;
        let y2 = b.dim().y * b.dim().y;
        let z2 = b.dim().z * b.dim().z;
        let mnorm = mass / S::from(12.).unwrap();
        let inertia = Matrix3::from_diagonal(Vector3::new(y2 + z2, x2 + z2, x2 + y2) * mnorm);
        Mass::new_with_inertia(mass, inertia)
    }
}

fn poly_sub_expr_calc<S>(w0: S, w1: S, w2: S) -> (S, S, S, S, S, S)
where
    S: BaseFloat,
{
    let t0 = w0 + w1;
    let t1 = w0 * w0;
    let t2 = t1 + t0 * w1;
    let f1 = t0 + w2;
    let f2 = t2 + w2 * f1;
    let f3 = w0 * t0 + w1 * t2 + w2 * f2;
    (
        f1,
        f2,
        f3,
        f2 + w0 * (f1 + w0),
        f2 + w1 * (f1 + w1),
        f2 + w2 * (f1 + w2),
    )
}

const ONE_6: f64 = 1. / 6.;
const ONE_24: f64 = 1. / 24.;
const ONE_60: f64 = 1. / 60.;
const ONE_120: f64 = 1. / 120.;
const POLY_SCALE: [f64; 10] = [
    ONE_6, ONE_24, ONE_24, ONE_24, ONE_60, ONE_60, ONE_60, ONE_120, ONE_120, ONE_120,
];

impl<S> Volume<S, Matrix3<S>> for ConvexPolyhedron<S>
where
    S: BaseFloat,
{
    // Volume of tetrahedron is 1/6 * a.cross(b).dot(c) where a = B - C, b = A - C, c = Origin - C
    // Sum for all faces
    fn get_mass(&self, material: &Material) -> Mass<S, Matrix3<S>> {
        let mut intg: [S; 10] = [S::zero(); 10];
        for (p0, p1, p2) in self.faces_iter() {
            let v1 = p1 - p0; // a1, b1, c1
            let v2 = p2 - p0; // a2, b2, c2
            let d = v1.cross(v2); // d0, d1, d2
            let (f1x, f2x, f3x, g0x, g1x, g2x) = poly_sub_expr_calc(p0.x, p1.x, p2.x);
            let (_, f2y, f3y, g0y, g1y, g2y) = poly_sub_expr_calc(p0.y, p1.y, p2.y);
            let (_, f2z, f3z, g0z, g1z, g2z) = poly_sub_expr_calc(p0.z, p1.z, p2.z);
            intg[0] += d.x * f1x;
            intg[1] += d.x * f2x;
            intg[2] += d.y * f2y;
            intg[3] += d.z * f2z;
            intg[4] += d.x * f3x;
            intg[5] += d.y * f3y;
            intg[6] += d.z * f3z;
            intg[7] += d.x * (p0.y * g0x + p1.y * g1x + p2.y * g2x);
            intg[8] += d.y * (p0.z * g0y + p1.z * g1y + p2.z * g2y);
            intg[9] += d.z * (p0.x * g0z + p1.x * g1z + p2.x * g2z);
        }
        for i in 0..10 {
            intg[i] *= S::from(POLY_SCALE[i]).unwrap();
        }
        let cm = Point3::new(intg[1] / intg[0], intg[2] / intg[0], intg[3] / intg[0]);
        let mut inertia = Matrix3::zero();
        inertia.x.x = intg[5] + intg[6] - intg[0] * (cm.y * cm.y + cm.z * cm.z);
        inertia.y.y = intg[4] + intg[6] - intg[0] * (cm.x * cm.x + cm.z * cm.z);
        inertia.z.z = intg[4] + intg[5] - intg[0] * (cm.x * cm.x + cm.y * cm.y);
        inertia.x.y = -(intg[7] - intg[0] * cm.x * cm.y);
        inertia.y.z = -(intg[8] - intg[0] * cm.y * cm.z);
        inertia.x.z = -(intg[9] - intg[0] * cm.x * cm.z);
        Mass::new_with_inertia(
            intg[0] * material.density(),
            inertia * material.density::<S>(),
        )
    }
}

impl<S> Volume<S, S> for Primitive2<S>
where
    S: BaseFloat + Inertia,
{
    fn get_mass(&self, material: &Material) -> Mass<S, S> {
        use collision::primitive::Primitive2::*;
        match *self {
            Particle(_) | Line(_) => Mass::new(material.density()),
            Circle(ref circle) => circle.get_mass(material),
            Rectangle(ref rectangle) => rectangle.get_mass(material),
            Square(ref square) => square.get_mass(material),
            ConvexPolygon(ref polygon) => polygon.get_mass(material),
        }
    }
}

impl<S> Volume<S, Matrix3<S>> for Capsule<S>
where
    S: BaseFloat,
{
    fn get_mass(&self, material: &Material) -> Mass<S, Matrix3<S>> {
        use std::f64::consts::PI;
        let pi = S::from(PI).unwrap();
        let rsq = self.radius() * self.radius();
        let hsq = self.height() * self.height();
        let two = S::one() + S::one();
        let three = two + S::one();
        let four = two + two;
        let five = three + two;
        let eight = five + three;
        let twelve = eight + four;
        let c_m = pi * rsq * self.height() * material.density();
        let h_m = pi * two / three * rsq * self.radius() * material.density();
        let mass = c_m + two * h_m;
        let c_i_xz = hsq / twelve + rsq / four;
        let h_i_xz = rsq * two / five + hsq / two + self.height() * self.radius() * three / eight;
        let i_xz = c_m * c_i_xz + h_m * h_i_xz * two;
        let i_y = c_m * rsq / two + h_m * rsq * four / five;
        let inertia = Matrix3::from_diagonal(Vector3::new(i_xz, i_y, i_xz));
        Mass::new_with_inertia(mass, inertia)
    }
}

impl<S> Volume<S, Matrix3<S>> for Cylinder<S>
where
    S: BaseFloat,
{
    fn get_mass(&self, material: &Material) -> Mass<S, Matrix3<S>> {
        use std::f64::consts::PI;
        let pi = S::from(PI).unwrap();
        let rsq = self.radius() * self.radius();
        let volume = pi * rsq * self.height();
        let mass = volume * material.density();
        let two = S::one() + S::one();
        let three = S::one() + two;
        let twelve = three * two * two;
        let i_y = mass * rsq / two;
        let i_xz = mass / twelve * (three * rsq + self.height() * self.height());
        let inertia = Matrix3::from_diagonal(Vector3::new(i_xz, i_y, i_xz));
        Mass::new_with_inertia(mass, inertia)
    }
}

impl<S> Volume<S, Matrix3<S>> for Primitive3<S>
where
    S: BaseFloat,
{
    fn get_mass(&self, material: &Material) -> Mass<S, Matrix3<S>> {
        use collision::primitive::Primitive3::*;
        match *self {
            Particle(_) | Quad(_) => Mass::new(material.density()),
            Sphere(ref sphere) => sphere.get_mass(material),
            Cuboid(ref cuboid) => cuboid.get_mass(material),
            Cube(ref cube) => cube.get_mass(material),
            Capsule(ref capsule) => capsule.get_mass(material),
            Cylinder(ref cylinder) => cylinder.get_mass(material),
            ConvexPolyhedron(ref polyhedra) => polyhedra.get_mass(material),
        }
    }
}

// Composite inertia : sum(I_i + M_i * d_i^2)
// I_i : Inertia of primitive with index i
// M_i : Mass of primitive with index i
// d_i : Offset from composite center of mass to primitive center of mass
impl<S, P, T, B, Y> Volume<S, S> for CollisionShape<P, T, B, Y>
where
    S: BaseFloat + Inertia,
    P: Volume<S, S> + Primitive<Point = Point2<S>> + ComputeBound<B>,
    B: Bound<Point = Point2<S>> + Clone + Union<B, Output = B>,
    T: Transform<Point2<S>>,
    Y: Default,
{
    fn get_mass(&self, material: &Material) -> Mass<S, S> {
        let (mass, inertia) = self
            .primitives()
            .iter()
            .map(|p| (p.0.get_mass(material), &p.1))
            .fold((S::zero(), S::zero()), |(a_m, a_i), (m, t)| {
                (a_m + m.mass(), a_i + m.local_inertia() + m.mass() * d2(t))
            });
        Mass::new_with_inertia(mass, inertia)
    }
}

fn d2<S, T>(t: &T) -> S
where
    S: BaseFloat,
    T: Transform<Point2<S>>,
{
    let p = t.transform_point(Point2::origin()).to_vec();
    p.dot(p)
}

impl<S, P, T, B, Y> Volume<S, Matrix3<S>> for CollisionShape<P, T, B, Y>
where
    S: BaseFloat,
    P: Volume<S, Matrix3<S>> + Primitive<Point = Point3<S>> + ComputeBound<B>,
    B: Bound<Point = Point3<S>> + Clone + Union<B, Output = B>,
    T: Transform<Point3<S>>,
    Y: Default,
{
    fn get_mass(&self, material: &Material) -> Mass<S, Matrix3<S>> {
        let (mass, inertia) = self
            .primitives()
            .iter()
            .map(|p| (p.0.get_mass(material), &p.1))
            .fold((S::zero(), Matrix3::zero()), |(a_m, a_i), (m, t)| {
                (a_m + m.mass(), a_i + m.local_inertia() + d3(t) * m.mass())
            });
        Mass::new_with_inertia(mass, inertia)
    }
}

fn d3<S, T>(t: &T) -> Matrix3<S>
where
    S: BaseFloat,
    T: Transform<Point3<S>>,
{
    let o = t.transform_point(Point3::origin()).to_vec();
    let d2 = o.magnitude2();
    let mut j = Matrix3::from_value(d2);
    j.x += o * -o.x;
    j.y += o * -o.y;
    j.z += o * -o.z;
    j
}
