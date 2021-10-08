use cgmath::assert_ulps_eq;
use cgmath::{InnerSpace, Point2, Point3};
use collision::{Line2, Line3, Ray, Ray2, Ray3};
use std::convert::TryFrom;

#[test]
fn test_ray_from_line_2d() {
    let line = Line2::new(Point2::new(1f32, 2.), Point2::new(3f32, 4.));
    assert!(Ray::try_from(&line).is_ok());
    if let Ok(ray) = Ray2::try_from(&line) {
        assert_ulps_eq!(ray.origin, &line.origin);
        assert_ulps_eq!(ray.direction, (&line.dest - &line.origin).normalize());
    }
    // A single point should not create a Ray
    let line = Line2::new(Point2::new(1., 2.), Point2::new(1., 2.));
    assert!(!Ray::try_from(&line).is_ok());
}

#[test]
fn test_ray_from_line_3d() {
    let line: Line3<f32> = Line3::new(Point3::new(1f32, 2., 3.), Point3::new(5f32, 56., 6.));
    assert!(Ray::try_from(&line).is_ok());
    if let Ok(ray) = Ray3::try_from(&line) {
        assert_ulps_eq!(ray.origin, &line.origin);
        assert_ulps_eq!(ray.direction, (&line.dest - &line.origin).normalize());
    }
    // A single point should not create a Ray
    let line = Line2::new(Point2::new(1., 2.), Point2::new(1., 2.));
    assert!(!Ray::try_from(&line).is_ok());
}
