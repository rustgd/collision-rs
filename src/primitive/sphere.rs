use cgmath::prelude::*;
use cgmath::{BaseFloat, Point3, Vector3};

use crate::prelude::*;
use crate::{Aabb3, Ray3};

/// Sphere primitive
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(
    feature = "serde",
    derive(Serialize, Deserialize),
    serde(crate = "serde_crate")
)]
pub struct Sphere<S> {
    /// Radius of the sphere
    pub radius: S,
}

impl<S> Sphere<S> {
    /// Create a new sphere primitive
    pub fn new(radius: S) -> Self {
        Self { radius }
    }
}

impl<S> Primitive for Sphere<S>
where
    S: BaseFloat,
{
    type Point = Point3<S>;

    fn support_point<T>(&self, direction: &Vector3<S>, transform: &T) -> Point3<S>
    where
        T: Transform<Point3<S>>,
    {
        let direction = transform.inverse_transform_vector(*direction).unwrap();
        transform.transform_point(Point3::from_vec(direction.normalize_to(self.radius)))
    }
}

impl<S> ComputeBound<Aabb3<S>> for Sphere<S>
where
    S: BaseFloat,
{
    fn compute_bound(&self) -> Aabb3<S> {
        Aabb3::new(
            Point3::from_value(-self.radius),
            Point3::from_value(self.radius),
        )
    }
}

impl<S> ComputeBound<crate::volume::Sphere<S>> for Sphere<S>
where
    S: BaseFloat,
{
    fn compute_bound(&self) -> crate::volume::Sphere<S> {
        crate::volume::Sphere {
            center: Point3::origin(),
            radius: self.radius,
        }
    }
}

impl<S> Discrete<Ray3<S>> for Sphere<S>
where
    S: BaseFloat,
{
    fn intersects(&self, r: &Ray3<S>) -> bool {
        let s = self;
        let l = Vector3::new(-r.origin.x, -r.origin.y, -r.origin.z);
        let tca = l.dot(r.direction);
        if tca < S::zero() {
            return false;
        }
        let d2 = l.dot(l) - tca * tca;
        d2 <= s.radius * s.radius
    }
}

impl<S> Continuous<Ray3<S>> for Sphere<S>
where
    S: BaseFloat,
{
    type Result = Point3<S>;

    fn intersection(&self, r: &Ray3<S>) -> Option<Point3<S>> {
        let s = self;

        let l = Vector3::new(-r.origin.x, -r.origin.y, -r.origin.z);
        let tca = l.dot(r.direction);
        if tca < S::zero() {
            return None;
        }
        let d2 = l.dot(l) - tca * tca;
        if d2 > s.radius * s.radius {
            return None;
        }
        let thc = (s.radius * s.radius - d2).sqrt();
        Some(r.origin + r.direction * (tca - thc))
    }
}

#[cfg(test)]
mod tests {
    use std;

    use cgmath::assert_ulps_eq;
    use cgmath::{Decomposed, Point3, Quaternion, Rad, Rotation3, Vector3};

    use super::*;

    // sphere
    #[test]
    fn test_sphere_support_1() {
        test_sphere_support(1., 0., 0., 10., 0., 0., 0.);
    }

    #[test]
    fn test_sphere_support_2() {
        test_sphere_support(
            1.,
            1.,
            1.,
            5.773_502_691_896_258,
            5.773_502_691_896_258,
            5.773_502_691_896_258,
            0.,
        );
    }

    #[test]
    fn test_sphere_support_3() {
        test_sphere_support(
            1.,
            0.,
            0.,
            10.,
            0.000_000_953_674_3,
            0.,
            -std::f32::consts::PI / 4.,
        );
    }

    #[test]
    fn test_sphere_support_4() {
        let sphere = Sphere::new(10.);
        let direction = Vector3::new(1., 0., 0.);
        let t = transform(0., 10., 0., 0.);
        let point = sphere.support_point(&direction, &t);
        assert_eq!(Point3::new(10., 10., 0.), point);
    }

    #[test]
    fn test_sphere_bound() {
        let sphere = Sphere::new(10.);
        assert_eq!(
            bound(-10., -10., -10., 10., 10., 10.),
            sphere.compute_bound()
        );
    }

    #[test]
    fn test_ray_discrete() {
        let sphere = Sphere::new(10.);
        let ray = Ray3::new(Point3::new(20., 0., 0.), Vector3::new(-1., 0., 0.));
        assert!(sphere.intersects(&ray));
        let ray = Ray3::new(Point3::new(20., 0., 0.), Vector3::new(1., 0., 0.));
        assert!(!sphere.intersects(&ray));
        let ray = Ray3::new(Point3::new(20., -15., 0.), Vector3::new(-1., 0., 0.));
        assert!(!sphere.intersects(&ray));
    }

    #[test]
    fn test_ray_discrete_transformed() {
        let sphere = Sphere::new(10.);
        let ray = Ray3::new(Point3::new(20., 0., 0.), Vector3::new(-1., 0., 0.));
        let t = transform(0., 0., 0., 0.);
        assert!(sphere.intersects_transformed(&ray, &t));
        let ray = Ray3::new(Point3::new(20., 0., 0.), Vector3::new(1., 0., 0.));
        assert!(!sphere.intersects_transformed(&ray, &t));
        let t = transform(0., 15., 0., 0.);
        let ray = Ray3::new(Point3::new(20., 0., 0.), Vector3::new(-1., 0., 0.));
        assert!(!sphere.intersects_transformed(&ray, &t));
    }

    #[test]
    fn test_ray_continuous() {
        let sphere = Sphere::new(10.);
        let ray = Ray3::new(Point3::new(20., 0., 0.), Vector3::new(-1., 0., 0.));
        assert_eq!(Some(Point3::new(10., 0., 0.)), sphere.intersection(&ray));
        let ray = Ray3::new(Point3::new(20., 0., 0.), Vector3::new(1., 0., 0.));
        assert_eq!(None, sphere.intersection(&ray));
        let ray = Ray3::new(Point3::new(20., -15., 0.), Vector3::new(-1., 0., 0.));
        assert_eq!(None, sphere.intersection(&ray));
    }

    #[test]
    fn test_ray_continuous_transformed() {
        let sphere = Sphere::new(10.);
        let ray = Ray3::new(Point3::new(20., 0., 0.), Vector3::new(-1., 0., 0.));
        let t = transform(0., 0., 0., 0.);
        assert_eq!(
            Some(Point3::new(10., 0., 0.)),
            sphere.intersection_transformed(&ray, &t)
        );
        let ray = Ray3::new(Point3::new(20., 0., 0.), Vector3::new(1., 0., 0.));
        assert_eq!(None, sphere.intersection_transformed(&ray, &t));
        let t = transform(0., 15., 0., 0.);
        let ray = Ray3::new(Point3::new(20., 0., 0.), Vector3::new(-1., 0., 0.));
        assert_eq!(None, sphere.intersection_transformed(&ray, &t));
    }

    fn test_sphere_support(dx: f32, dy: f32, dz: f32, px: f32, py: f32, pz: f32, rot: f32) {
        let sphere = Sphere::new(10.);
        let direction = Vector3::new(dx, dy, dz);
        let t = transform(0., 0., 0., rot);
        let point = sphere.support_point(&direction, &t);
        assert_ulps_eq!(px, point.x);
        assert_ulps_eq!(py, point.y);
        assert_ulps_eq!(pz, point.z);
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
