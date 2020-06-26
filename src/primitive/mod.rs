//! Collision primitives

pub use self::capsule::Capsule;
pub use self::circle::Circle;
pub use self::cuboid::{Cube, Cuboid};
pub use self::cylinder::Cylinder;
pub use self::particle::*;
pub use self::polygon::ConvexPolygon;
pub use self::polyhedron::ConvexPolyhedron;
pub use self::primitive2::Primitive2;
pub use self::primitive3::Primitive3;
pub use self::quad::Quad;
pub use self::rectangle::{Rectangle, Square};
pub use self::sphere::Sphere;

mod capsule;
mod circle;
mod cuboid;
mod cylinder;
mod line;
mod particle;
mod polygon;
mod polyhedron;
mod primitive2;
mod primitive3;
mod quad;
mod rectangle;
mod sphere;

pub(crate) mod util;

use cgmath::{EuclideanSpace, Transform};

use crate::prelude::*;

impl<B, P> HasBound for (P, B)
where
    P: Primitive,
    B: Bound,
{
    type Bound = B;

    fn bound(&self) -> &Self::Bound {
        &self.1
    }
}

impl<B, P> Primitive for (P, B)
where
    P: Primitive,
    B: Bound,
{
    type Point = P::Point;

    fn support_point<T>(
        &self,
        direction: &<Self::Point as EuclideanSpace>::Diff,
        transform: &T,
    ) -> Self::Point
    where
        T: Transform<Self::Point>,
    {
        self.0.support_point(direction, transform)
    }
}
