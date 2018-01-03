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

use cgmath::{EuclideanSpace, Transform};

use prelude::*;

impl<B, P> HasBound for (P, B)
where
    P: Primitive,
    B: BoundingVolume,
{
    type Bound = B;

    fn bound(&self) -> &Self::Bound {
        &self.1
    }
}

impl<B, P> SupportFunction for (P, B)
where
    P: Primitive,
    B: BoundingVolume,
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
