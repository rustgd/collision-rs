//! Rectangular plane primitive

use cgmath::prelude::*;
use cgmath::{BaseFloat, Point3, Vector2, Vector3};

use crate::prelude::*;
use crate::primitive::util::get_max_point;
use crate::{Aabb3, Ray3, Sphere};

/// Rectangular plane primitive. Will lie on the xy plane when not transformed.
///
/// Have a cached set of corner points to speed up computation.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(
    feature = "serde",
    derive(Serialize, Deserialize),
    serde(crate = "serde_crate")
)]
pub struct Quad<S> {
    /// Dimensions of the rectangle
    dim: Vector2<S>,
    half_dim: Vector2<S>,
    corners: [Point3<S>; 4],
}

impl<S> Quad<S>
where
    S: BaseFloat,
{
    /// Create a new rectangle primitive from component dimensions
    pub fn new(dim_x: S, dim_y: S) -> Self {
        Self::new_impl(Vector2::new(dim_x, dim_y))
    }

    /// Create a new rectangle primitive from a vector of component dimensions
    pub fn new_impl(dim: Vector2<S>) -> Self {
        let half_dim = dim / (S::one() + S::one());
        Quad {
            dim,
            half_dim,
            corners: Self::generate_corners(&half_dim),
        }
    }

    /// Get the dimensions of the `Rectangle`
    pub fn dim(&self) -> &Vector2<S> {
        &self.dim
    }

    /// Get the half dimensions of the `Rectangle`
    pub fn half_dim(&self) -> &Vector2<S> {
        &self.half_dim
    }

    fn generate_corners(half_dim: &Vector2<S>) -> [Point3<S>; 4] {
        [
            Point3::new(half_dim.x, half_dim.y, S::zero()),
            Point3::new(-half_dim.x, half_dim.y, S::zero()),
            Point3::new(-half_dim.x, -half_dim.y, S::zero()),
            Point3::new(half_dim.x, -half_dim.y, S::zero()),
        ]
    }
}

impl<S> Primitive for Quad<S>
where
    S: BaseFloat,
{
    type Point = Point3<S>;

    fn support_point<T>(&self, direction: &Vector3<S>, transform: &T) -> Point3<S>
    where
        T: Transform<Point3<S>>,
    {
        get_max_point(self.corners.iter(), direction, transform)
    }
}

impl<S> ComputeBound<Aabb3<S>> for Quad<S>
where
    S: BaseFloat,
{
    fn compute_bound(&self) -> Aabb3<S> {
        Aabb3::new(
            Point3::new(-self.half_dim.x, -self.half_dim.y, S::zero()),
            Point3::new(self.half_dim.x, self.half_dim.y, S::zero()),
        )
    }
}

impl<S> ComputeBound<Sphere<S>> for Quad<S>
where
    S: BaseFloat,
{
    fn compute_bound(&self) -> Sphere<S> {
        Sphere {
            center: Point3::origin(),
            radius: self.half_dim.x.max(self.half_dim.y),
        }
    }
}

impl<S> Discrete<Ray3<S>> for Quad<S>
where
    S: BaseFloat,
{
    /// Ray must be in object space of the rectangle
    fn intersects(&self, ray: &Ray3<S>) -> bool {
        let aabb: Aabb3<S> = self.compute_bound();
        aabb.intersects(ray)
    }
}

impl<S> Continuous<Ray3<S>> for Quad<S>
where
    S: BaseFloat,
{
    type Result = Point3<S>;

    /// Ray must be in object space of the rectangle
    fn intersection(&self, ray: &Ray3<S>) -> Option<Point3<S>> {
        let aabb: Aabb3<S> = self.compute_bound();
        aabb.intersection(ray)
    }
}

#[cfg(test)]
mod tests {

    use super::*;
    use crate::algorithm::minkowski::GJK3;
    use crate::primitive::Cuboid;
    use cgmath::{Decomposed, Quaternion};

    fn transform(x: f32, y: f32, z: f32) -> Decomposed<Vector3<f32>, Quaternion<f32>> {
        Decomposed {
            disp: Vector3::new(x, y, z),
            rot: Quaternion::one(),
            scale: 1.,
        }
    }

    #[test]
    fn test_plane_cuboid_intersect() {
        let quad = Quad::new(2., 2.);
        let cuboid = Cuboid::new(1., 1., 1.);
        let transform_1 = transform(0., 0., 1.);
        let transform_2 = transform(0., 0., 1.1);
        let gjk = GJK3::new();
        assert!(gjk
            .intersect(&quad, &transform_1, &cuboid, &transform_2)
            .is_some());
    }
}
