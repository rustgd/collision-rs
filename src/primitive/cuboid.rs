use cgmath::{Point3, Vector3, BaseFloat};
use cgmath::prelude::*;

use {Aabb3, Ray3};
use prelude::*;
use traits::{HasAABB, SupportFunction, ContinuousTransformed, DiscreteTransformed};

/// Cuboid primitive.
///
/// Have a cached set of corner points to speed up computation.
#[derive(Debug, Clone)]
pub struct Cuboid<S> {
    /// Dimensions of the box
    pub dim: Vector3<S>,
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
        Self {
            dim,
            half_dim: dim / (S::one() + S::one()),
            corners: Self::generate_corners(&dim),
        }
    }

    fn generate_corners(dimensions: &Vector3<S>) -> Vec<Point3<S>> {
        let two = S::one() + S::one();
        vec![
            Point3::new(dimensions.x, dimensions.y, dimensions.z) / two,
            Point3::new(-dimensions.x, dimensions.y, dimensions.z) / two,
            Point3::new(-dimensions.x, -dimensions.y, dimensions.z) / two,
            Point3::new(dimensions.x, -dimensions.y, dimensions.z) / two,
            Point3::new(dimensions.x, dimensions.y, -dimensions.z) / two,
            Point3::new(-dimensions.x, dimensions.y, -dimensions.z) / two,
            Point3::new(-dimensions.x, -dimensions.y, -dimensions.z) / two,
            Point3::new(dimensions.x, -dimensions.y, -dimensions.z) / two,
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
        ::primitive::util::get_max_point(&self.corners, direction, transform)
    }
}

impl<S> HasAABB for Cuboid<S>
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

impl<S> DiscreteTransformed<Ray3<S>> for Cuboid<S>
where
    S: BaseFloat,
{
    type Point = Point3<S>;

    fn intersects_transformed<T>(&self, ray: &Ray3<S>, transform: &T) -> bool
    where
        T: Transform<Point3<S>>,
    {
        self.intersects(&ray.transform(transform.inverse_transform().unwrap()))
    }
}

impl<S> Discrete<Ray3<S>> for Cuboid<S>
where
    S: BaseFloat,
{
    fn intersects(&self, ray: &Ray3<S>) -> bool {
        let min = Point3::new(-self.half_dim.x, -self.half_dim.y, -self.half_dim.z);
        let max = Point3::new(self.half_dim.x, self.half_dim.y, self.half_dim.z);

        let inv_dir = Vector3::from_value(S::one()).div_element_wise(ray.direction);

        let mut t1 = (min.x - ray.origin.x) * inv_dir.x;
        let mut t2 = (max.x - ray.origin.x) * inv_dir.x;

        let mut tmin = t1.min(t2);
        let mut tmax = t1.max(t2);

        for i in 1..3 {
            t1 = (min[i] - ray.origin[i]) * inv_dir[i];
            t2 = (max[i] - ray.origin[i]) * inv_dir[i];

            tmin = tmin.max(t1.min(t2));
            tmax = tmax.min(t1.max(t2));
        }

        tmax >= tmin && (tmin >= S::zero() || tmax >= S::zero())
    }
}

impl<S> ContinuousTransformed<Ray3<S>> for Cuboid<S>
where
    S: BaseFloat,
{
    type Point = Point3<S>;
    type Result = Point3<S>;

    fn intersection_transformed<T>(&self, ray: &Ray3<S>, transform: &T) -> Option<Point3<S>>
    where
        T: Transform<Point3<S>>,
    {
        self.intersection(&ray.transform(transform.inverse_transform().unwrap()))
            .map(|p| transform.transform_point(p))
    }
}

impl<S> Continuous<Ray3<S>> for Cuboid<S>
where
    S: BaseFloat,
{
    type Result = Point3<S>;

    fn intersection(&self, ray: &Ray3<S>) -> Option<Point3<S>> {
        let min = Point3::new(-self.half_dim.x, -self.half_dim.y, -self.half_dim.z);
        let max = Point3::new(self.half_dim.x, self.half_dim.y, self.half_dim.z);

        let inv_dir = Vector3::from_value(S::one()).div_element_wise(ray.direction);

        let mut t1 = (min.x - ray.origin.x) * inv_dir.x;
        let mut t2 = (max.x - ray.origin.x) * inv_dir.x;

        let mut tmin = t1.min(t2);
        let mut tmax = t1.max(t2);

        for i in 1..3 {
            t1 = (min[i] - ray.origin[i]) * inv_dir[i];
            t2 = (max[i] - ray.origin[i]) * inv_dir[i];

            tmin = tmin.max(t1.min(t2));
            tmax = tmax.min(t1.max(t2));
        }

        if (tmin < S::zero() && tmax < S::zero()) || tmax < tmin {
            None
        } else {
            let t = if tmin >= S::zero() { tmin } else { tmax };
            Some(ray.origin + ray.direction * t)
        }
    }
}

#[cfg(test)]
mod tests {

    use cgmath::{Point3, Vector3, Quaternion, Rad, Decomposed};

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
