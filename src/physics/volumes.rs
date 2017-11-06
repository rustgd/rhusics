use cgmath::Transform;
use collision::{Aabb, HasAabb, Primitive};
use collision::primitive::{Primitive2, Primitive3};

use super::{Mass, Material, Volume};
use Real;
use collide::CollisionShape;

// FIXME: inertia
impl Volume<Real> for Primitive2<Real> {
    fn get_mass(&self, material: &Material) -> Mass<Real> {
        use collision::primitive::Primitive2::*;
        match *self {
            Particle(_) => Mass::new(material.density()),
            Circle(ref circle) => {
                use std::f64::consts::PI;
                let pi = PI as Real;
                Mass::new(pi * circle.radius * circle.radius * material.density())
            }
            Rectangle(ref rectangle) => {
                Mass::new(rectangle.get_bound().volume() * material.density())
            }
            // FIXME: use actual polygon data
            ConvexPolygon(ref polygon) => {
                Mass::new(polygon.get_bound().volume() * material.density())
            }
        }
    }
}

// FIXME: inertia
impl Volume<Real> for Primitive3<Real> {
    fn get_mass(&self, material: &Material) -> Mass<Real> {
        use collision::primitive::Primitive3::*;
        match *self {
            Particle(_) => Mass::new(material.density()),
            Sphere(ref sphere) => {
                use std::f64::consts::PI;
                let pi = PI as Real;
                Mass::new(
                    4. / 3. * pi * sphere.radius * sphere.radius * sphere.radius
                        * material.density(),
                )
            }
            Cuboid(ref cuboid) => Mass::new(cuboid.get_bound().volume() * material.density()),
            // FIXME: use actual mesh data
            ConvexPolyhedron(ref polyhedra) => {
                Mass::new(polyhedra.get_bound().volume() * material.density())
            }
        }
    }
}

// FIXME: inertia
impl<P, T> Volume<Real> for CollisionShape<P, T>
where
    P: Volume<Real> + Primitive,
    P::Aabb: Aabb<Scalar = Real>,
    T: Transform<P::Point>,
{
    fn get_mass(&self, material: &Material) -> Mass<Real> {
        Mass::new(
            self.primitives()
                .iter()
                .map(|p| p.0.get_mass(material))
                .fold(0., |a, m| a + m.mass),
        )
    }
}
