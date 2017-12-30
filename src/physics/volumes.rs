use cgmath::{EuclideanSpace, InnerSpace, Matrix3, Point2, Point3, SquareMatrix, Transform,
             Vector3, Zero};
use collision::{Aabb, Aabb2, Aabb3, Primitive, BoundingVolume, Union};
use collision::primitive::*;

use super::{Mass, Material, PartialCrossProduct};
use Real;
use collide::CollisionShape;

/// Describe a shape with volume
///
/// ### Type parameters:
///
/// - `I`: Inertia type, see `Inertia` for more information
pub trait Volume<I> {
    /// Compute the mass of the shape based on its material
    fn get_mass(&self, material: &Material) -> Mass<I>;
}

impl Volume<Real> for Circle<Real> {
    fn get_mass(&self, material: &Material) -> Mass<Real> {
        use std::f64::consts::PI;
        let pi = PI as Real;
        let mass = pi * self.radius * self.radius * material.density();
        let inertia = mass * self.radius * self.radius / 2.;
        Mass::new_with_inertia(mass, inertia)
    }
}

impl Volume<Real> for Rectangle<Real> {
    fn get_mass(&self, material: &Material) -> Mass<Real> {
        let b = Aabb2::from(self);
        let mass = b.volume() * material.density();
        let inertia = mass * (b.dim().x * b.dim().x + b.dim().y * b.dim().y) / 12.;
        Mass::new_with_inertia(mass, inertia)
    }
}

impl Volume<Real> for ConvexPolygon<Real> {
    fn get_mass(&self, material: &Material) -> Mass<Real> {
        let mut area = 0. as Real;
        let mut denom = 0.;
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
        let mass = area * 0.5 * material.density();
        let inertia = mass / 6. * denom / area;
        Mass::new_with_inertia(mass, inertia)
    }
}

impl Volume<Matrix3<Real>> for Sphere<Real> {
    fn get_mass(&self, material: &Material) -> Mass<Matrix3<Real>> {
        use std::f64::consts::PI;
        let pi = PI as Real;
        let mass = 4. / 3. * pi * self.radius * self.radius * self.radius * material.density();
        let inertia = 2. / 5. * mass * self.radius * self.radius;
        Mass::new_with_inertia(mass, Matrix3::from_value(inertia))
    }
}

impl Volume<Matrix3<Real>> for Cuboid<Real> {
    fn get_mass(&self, material: &Material) -> Mass<Matrix3<Real>> {
        let b = Aabb3::from(self);
        let mass = b.volume() * material.density();
        let x2 = b.dim().x * b.dim().x;
        let y2 = b.dim().y * b.dim().y;
        let z2 = b.dim().z * b.dim().z;
        let mnorm = mass / 12.;
        let inertia = Matrix3::from_diagonal(Vector3::new(y2 + z2, x2 + z2, x2 + y2) * mnorm);
        Mass::new_with_inertia(mass, inertia)
    }
}

fn poly_sub_expr_calc(w0: Real, w1: Real, w2: Real) -> (Real, Real, Real, Real, Real, Real) {
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

const ONE_6: Real = 1. / 6.;
const ONE_24: Real = 1. / 24.;
const ONE_60: Real = 1. / 60.;
const ONE_120: Real = 1. / 120.;
const POLY_SCALE: [Real; 10] = [
    ONE_6, ONE_24, ONE_24, ONE_24, ONE_60, ONE_60, ONE_60, ONE_120, ONE_120, ONE_120
];

impl Volume<Matrix3<Real>> for ConvexPolyhedron<Real> {
    // Volume of tetrahedron is 1/6 * a.cross(b).dot(c) where a = B - C, b = A - C, c = Origin - C
    // Sum for all faces
    fn get_mass(&self, material: &Material) -> Mass<Matrix3<Real>> {
        let mut intg: [Real; 10] = [0.; 10];
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
            intg[i] *= POLY_SCALE[i];
        }
        let cm = Point3::new(intg[1] / intg[0], intg[2] / intg[0], intg[3] / intg[0]);
        let mut inertia = Matrix3::zero();
        inertia.x.x = intg[5] + intg[6] - intg[0] * (cm.y * cm.y + cm.z * cm.z);
        inertia.y.y = intg[4] + intg[6] - intg[0] * (cm.x * cm.x + cm.z * cm.z);
        inertia.z.z = intg[4] + intg[5] - intg[0] * (cm.x * cm.x + cm.y * cm.y);
        inertia.x.y = -(intg[7] - intg[0] * cm.x * cm.y);
        inertia.y.z = -(intg[8] - intg[0] * cm.y * cm.z);
        inertia.x.z = -(intg[9] - intg[0] * cm.x * cm.z);
        Mass::new_with_inertia(intg[0] * material.density(), inertia * material.density())
    }
}

impl Volume<Real> for Primitive2<Real> {
    fn get_mass(&self, material: &Material) -> Mass<Real> {
        use collision::primitive::Primitive2::*;
        match *self {
            Particle(_) => Mass::new(material.density()),
            Circle(ref circle) => circle.get_mass(material),
            Rectangle(ref rectangle) => rectangle.get_mass(material),
            ConvexPolygon(ref polygon) => polygon.get_mass(material),
        }
    }
}

impl Volume<Matrix3<Real>> for Capsule<Real> {
    fn get_mass(&self, material: &Material) -> Mass<Matrix3<Real>> {
        use std::f64::consts::PI;
        let pi = PI as Real;
        let rsq = self.radius() * self.radius();
        let hsq = self.height() * self.height();
        let c_m = pi * rsq * self.height() * material.density();
        let h_m = pi * 2. / 3. * rsq * self.radius() * material.density();
        let mass = c_m + 2. * h_m;
        let c_i_xz = hsq / 12. + rsq / 4.;
        let h_i_xz = rsq * 2. / 5. + hsq / 2. + self.height() * self.radius() * 3. / 8.;
        let i_xz = c_m * c_i_xz + h_m * h_i_xz * 2.;
        let i_y = c_m * rsq / 2. + h_m * rsq * 4. / 5.;
        let inertia = Matrix3::from_diagonal(Vector3::new(i_xz, i_y, i_xz));
        Mass::new_with_inertia(mass, inertia)
    }
}

impl Volume<Matrix3<Real>> for Cylinder<Real> {
    fn get_mass(&self, material: &Material) -> Mass<Matrix3<Real>> {
        use std::f64::consts::PI;
        let pi = PI as Real;
        let rsq = self.radius() * self.radius();
        let volume = pi * rsq * self.height();
        let mass = volume * material.density();
        let i_y = mass * rsq / 2.;
        let i_xz = mass / 12. * (3. * rsq + self.height() * self.height());
        let inertia = Matrix3::from_diagonal(Vector3::new(i_xz, i_y, i_xz));
        Mass::new_with_inertia(mass, inertia)
    }
}

impl Volume<Matrix3<Real>> for Primitive3<Real> {
    fn get_mass(&self, material: &Material) -> Mass<Matrix3<Real>> {
        use collision::primitive::Primitive3::*;
        match *self {
            Particle(_) => Mass::new(material.density()),
            Sphere(ref sphere) => sphere.get_mass(material),
            Cuboid(ref cuboid) => cuboid.get_mass(material),
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
impl<P, T, B, Y> Volume<Real> for CollisionShape<P, T, B, Y>
where
    P: Volume<Real> + Primitive<Point = Point2<Real>>,
    for <'a> B: From<&'a P>,
    B: BoundingVolume<Point = Point2<Real>> + Clone + Union<B, Output=B>,
    T: Transform<Point2<Real>>,
    Y: Default,
{
    fn get_mass(&self, material: &Material) -> Mass<Real> {
        let (mass, inertia) = self.primitives()
            .iter()
            .map(|p| (p.0.get_mass(material), &p.1))
            .fold((0., 0.), |(a_m, a_i), (m, t)| {
                (a_m + m.mass(), a_i + m.local_inertia() + m.mass() * d2(t))
            });
        Mass::new_with_inertia(mass, inertia)
    }
}

fn d2<T>(t: &T) -> Real
where
    T: Transform<Point2<Real>>,
{
    let p = t.transform_point(Point2::origin()).to_vec();
    p.dot(p)
}

impl<P, T, B, Y> Volume<Matrix3<Real>> for CollisionShape<P, T, B, Y>
where
    P: Volume<Matrix3<Real>> + Primitive<Point = Point3<Real>>,
    for <'a> B: From<&'a P>,
    B: BoundingVolume<Point = Point3<Real>> + Clone + Union<B, Output=B>,
    T: Transform<Point3<Real>>,
    Y: Default,
{
    fn get_mass(&self, material: &Material) -> Mass<Matrix3<Real>> {
        let (mass, inertia) = self.primitives()
            .iter()
            .map(|p| (p.0.get_mass(material), &p.1))
            .fold((0., Matrix3::zero()), |(a_m, a_i), (m, t)| {
                (a_m + m.mass(), a_i + m.local_inertia() + d3(t) * m.mass())
            });
        Mass::new_with_inertia(mass, inertia)
    }
}

fn d3<T>(t: &T) -> Matrix3<Real>
where
    T: Transform<Point3<Real>>,
{
    let o = t.transform_point(Point3::origin()).to_vec();
    let d2 = o.magnitude2();
    let mut j = Matrix3::from_value(d2);
    j.x += o * -o.x;
    j.y += o * -o.y;
    j.z += o * -o.z;
    j
}
