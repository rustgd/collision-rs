use cgmath::assert_ulps_eq;
use cgmath::{Point3, Vector3};
use collision::{Continuous, Plane, PlaneBound, Ray3, Relation};

#[test]
fn test_homogeneous() {
    let p = Point3::new(1.0f64, 2.0f64, 3.0f64);
    assert_ulps_eq!(p, &Point3::from_homogeneous(p.to_homogeneous()));
}

#[test]
fn test_bound() {
    let point = Point3::new(1f32, 2.0, 3.0);
    let normal = Vector3::new(0f32, -0.8, -0.36);
    let plane = Plane::from_point_normal(point, normal);

    assert_eq!(point.relate_plane(plane), Relation::Cross);
    assert_eq!((point + normal).relate_plane(plane), Relation::In);
    assert_eq!((point + normal * -1.0).relate_plane(plane), Relation::Out);
}

#[test]
fn test_ray_intersect() {
    let point = Point3::new(1f32, 2.0, 3.0);
    // ray across the point
    let ray1 = Ray3::new(Point3::new(1f32, 2.0, 0.0), Vector3::new(0.0, 0.0, 1.0));
    assert_eq!(point.intersection(&ray1), Some(point));
    // ray in the opposite direction
    let ray2 = Ray3::new(Point3::new(1f32, 2.0, 0.0), Vector3::new(0.0, 0.0, -1.0));
    assert_eq!(point.intersection(&ray2), None);
    // unrelated ray
    let ray3 = Ray3::new(Point3::new(1f32, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0));
    assert_eq!(point.intersection(&ray3), None);
}
