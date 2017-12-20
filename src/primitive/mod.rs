//! Collision primitives

pub use self::capsule::Capsule;
pub use self::circle::Circle;
pub use self::cuboid::Cuboid;
pub use self::cylinder::Cylinder;
pub use self::particle::*;
pub use self::polygon::ConvexPolygon;
pub use self::polyhedron::ConvexPolyhedron;
pub use self::primitive2::Primitive2;
pub use self::primitive3::Primitive3;
pub use self::rectangle::Rectangle;
pub use self::sphere::Sphere;

mod circle;
mod rectangle;
mod polygon;
mod sphere;
mod cylinder;
mod capsule;
mod cuboid;
mod polyhedron;
mod particle;
mod primitive2;
mod primitive3;

pub(crate) mod util;
