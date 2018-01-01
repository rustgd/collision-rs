use cgmath::{BaseFloat, Point3, Vector3};
use cgmath::prelude::*;

use {Aabb3, Ray3};
use prelude::*;
use primitive::util::get_max_point;
use volume::Sphere;

/// Cuboid primitive.
///
/// Have a cached set of corner points to speed up computation.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "eders", derive(Serialize, Deserialize))]
pub struct Cuboid<S> {
    /// Dimensions of the box
    dim: Vector3<S>,
    half_dim: Vector3<S>,
    corners: Vec<Point3<S>>,
}

impl<S> Cuboid<S>
where
    S: BaseFloat,
{
    /// Create a new rectangle primitive from component dimensions
    pub fn new(dim_x: S, dim_y: S, dim_z: S) -> Self {
        Self::new_impl(Vector3::new(dim_x, dim_y, dim_z))
    }

    /// Create a new rectangle primitive from a vector of component dimensions
    pub fn new_impl(dim: Vector3<S>) -> Self {
        let half_dim = dim / (S::one() + S::one());
        Self {
            dim,
            half_dim,
            corners: Self::generate_corners(&half_dim),
        }
    }

    fn generate_corners(half_dim: &Vector3<S>) -> Vec<Point3<S>> {
        vec![
            Point3::new(half_dim.x, half_dim.y, half_dim.z),
            Point3::new(-half_dim.x, half_dim.y, half_dim.z),
            Point3::new(-half_dim.x, -half_dim.y, half_dim.z),
            Point3::new(half_dim.x, -half_dim.y, half_dim.z),
            Point3::new(half_dim.x, half_dim.y, -half_dim.z),
            Point3::new(-half_dim.x, half_dim.y, -half_dim.z),
            Point3::new(-half_dim.x, -half_dim.y, -half_dim.z),
            Point3::new(half_dim.x, -half_dim.y, -half_dim.z),
        ]
    }
}

impl<S> SupportFunction for Cuboid<S>
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

impl<'a, S> From<&'a Cuboid<S>> for Aabb3<S>
where
    S: BaseFloat,
{
    fn from(cuboid: &Cuboid<S>) -> Self {
        Aabb3::new(
            Point3::from_vec(-cuboid.half_dim),
            Point3::from_vec(cuboid.half_dim),
        )
    }
}

impl<'a, S> From<&'a Cuboid<S>> for Sphere<S>
where
    S: BaseFloat,
{
    fn from(cuboid: &Cuboid<S>) -> Self {
        let max = cuboid
            .half_dim
            .x
            .max(cuboid.half_dim.y)
            .max(cuboid.half_dim.z);
        Sphere {
            center: Point3::origin(),
            radius: max,
        }
    }
}

impl<S> HasAabb for Cuboid<S>
where
    S: BaseFloat,
{
    type Aabb = Aabb3<S>;

    fn get_bound(&self) -> Aabb3<S> {
        Aabb3::new(
            Point3::from_vec(-self.half_dim),
            Point3::from_vec(self.half_dim),
        )
    }
}

impl<S> Discrete<Ray3<S>> for Cuboid<S>
where
    S: BaseFloat,
{
    fn intersects(&self, ray: &Ray3<S>) -> bool {
        self.get_bound().intersects(ray)
    }
}

impl<S> Continuous<Ray3<S>> for Cuboid<S>
where
    S: BaseFloat,
{
    type Result = Point3<S>;

    fn intersection(&self, ray: &Ray3<S>) -> Option<Point3<S>> {
        self.get_bound().intersection(ray)
    }
}

#[cfg(test)]
mod tests {

    use cgmath::{Decomposed, Point3, Quaternion, Rad, Vector3};

    use super::*;
    use Ray3;

    #[test]
    fn test_rectangle_bound() {
        let r = Cuboid::new(10., 10., 10.);
        assert_eq!(bound(-5., -5., -5., 5., 5., 5.), r.get_bound())
    }

    #[test]
    fn test_ray_discrete() {
        let cuboid = Cuboid::new(10., 10., 10.);
        let ray = Ray3::new(Point3::new(10., 0., 0.), Vector3::new(-1., 0., 0.));
        assert!(cuboid.intersects(&ray));
        let ray = Ray3::new(Point3::new(10., 0., 0.), Vector3::new(1., 0., 0.));
        assert!(!cuboid.intersects(&ray));
    }

    #[test]
    fn test_ray_discrete_transformed() {
        let cuboid = Cuboid::new(10., 10., 10.);
        let ray = Ray3::new(Point3::new(10., 0., 0.), Vector3::new(-1., 0., 0.));
        let t = transform(0., 1., 0., 0.);
        assert!(cuboid.intersects_transformed(&ray, &t));
        let ray = Ray3::new(Point3::new(10., 0., 0.), Vector3::new(1., 0., 0.));
        assert!(!cuboid.intersects_transformed(&ray, &t));
        let ray = Ray3::new(Point3::new(10., 0., 0.), Vector3::new(-1., 0., 0.));
        let t = transform(0., 1., 0., 0.3);
        assert!(cuboid.intersects_transformed(&ray, &t));
    }

    #[test]
    fn test_ray_continuous() {
        let cuboid = Cuboid::new(10., 10., 10.);
        let ray = Ray3::new(Point3::new(10., 0., 0.), Vector3::new(-1., 0., 0.));
        assert_eq!(Some(Point3::new(5., 0., 0.)), cuboid.intersection(&ray));
        let ray = Ray3::new(Point3::new(10., 0., 0.), Vector3::new(1., 0., 0.));
        assert_eq!(None, cuboid.intersection(&ray));
    }

    #[test]
    fn test_ray_continuous_transformed() {
        let cuboid = Cuboid::new(10., 10., 10.);
        let ray = Ray3::new(Point3::new(10., 0., 0.), Vector3::new(-1., 0., 0.));
        let t = transform(0., 1., 0., 0.);
        assert_eq!(
            Some(Point3::new(5., 0., 0.)),
            cuboid.intersection_transformed(&ray, &t)
        );
        let ray = Ray3::new(Point3::new(10., 0., 0.), Vector3::new(1., 0., 0.));
        assert_eq!(None, cuboid.intersection_transformed(&ray, &t));
        let ray = Ray3::new(Point3::new(10., 0., 0.), Vector3::new(-1., 0., 0.));
        let t = transform(0., 0., 0., 0.3);
        let p = cuboid.intersection_transformed(&ray, &t).unwrap();
        assert_ulps_eq!(5.233758, p.x);
        assert_ulps_eq!(0., p.y);
        assert_ulps_eq!(0., p.z);
    }

    // util
    fn transform(dx: f32, dy: f32, dz: f32, rot: f32) -> Decomposed<Vector3<f32>, Quaternion<f32>> {
        Decomposed {
            scale: 1.,
            rot: Quaternion::from_angle_z(Rad(rot)),
            disp: Vector3::new(dx, dy, dz),
        }
    }

    fn bound(min_x: f32, min_y: f32, min_z: f32, max_x: f32, max_y: f32, max_z: f32) -> Aabb3<f32> {
        Aabb3::new(
            Point3::new(min_x, min_y, min_z),
            Point3::new(max_x, max_y, max_z),
        )
    }
}
