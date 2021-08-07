use cgmath::*;
use collision::*;

#[test]
fn test_from_points() {
    assert_eq!(
        Plane::from_points(
            Point3::new(5.0f64, 0.0f64, 5.0f64),
            Point3::new(5.0f64, 5.0f64, 5.0f64),
            Point3::new(5.0f64, 0.0f64, -1.0f64),
        ),
        Some(Plane::from_abcd(-1.0f64, 0.0f64, 0.0f64, 5.0f64))
    );

    assert_eq!(
        Plane::from_points(
            Point3::new(0.0f64, 5.0f64, -5.0f64),
            Point3::new(0.0f64, 5.0f64, 0.0f64),
            Point3::new(0.0f64, 5.0f64, 5.0f64),
        ),
        None
    ); // The points are parallel
}

#[test]
fn test_ray_intersection() {
    let p0 = Plane::from_abcd(1f64, 0f64, 0f64, -7f64);
    let r0: Ray3<f64> = Ray::new(
        Point3::new(2f64, 3f64, 4f64),
        Vector3::new(1f64, 1f64, 1f64).normalize(),
    );
    assert!(p0.intersects(&r0));
    assert_eq!(p0.intersection(&r0), Some(Point3::new(7f64, 8f64, 9f64)));

    let p1 = Plane::from_points(
        Point3::new(5f64, 0f64, 5f64),
        Point3::new(5f64, 5f64, 5f64),
        Point3::new(5f64, 0f64, -1f64),
    )
    .unwrap();
    let r1: Ray3<f64> = Ray::new(
        Point3::new(0f64, 0f64, 0f64),
        Vector3::new(-1f64, 0f64, 0f64).normalize(),
    );
    assert_eq!(p1.intersection(&r1), None); // r1 points away from p1
    assert!(!p1.intersects(&r1));
}

#[test]
fn test_plane2_intersection() {
    let p0 = Plane::new(Vector3::unit_x(), 1.0f64);
    let p1 = Plane::new(Vector3::unit_y(), 2.0f64);
    let ray = p0.intersection(&p1);
    assert!(ray.is_some());
    assert!(p0.intersects(&p1));

    let ray = ray.unwrap();
    assert_ulps_eq!(ray.origin.x, &1.0f64);
    assert_ulps_eq!(ray.origin.y, &2.0f64);
    assert_ulps_eq!(ray.origin.z, &0.0f64);
    assert_ulps_eq!(ray.direction.x, &0.0f64);
    assert_ulps_eq!(ray.direction.y, &0.0f64);
    assert_ulps_eq!(ray.direction.z, &1.0f64);

    let p0 = Plane::new(Vector3::unit_y(), 1.0f64);
    let p1 = Plane::new(Vector3::unit_y(), 2.0f64);
    let ray = p0.intersection(&p1);
    assert!(ray.is_none());
    assert!(!p0.intersects(&p1));
}

#[test]
fn test_plane3_intersection() {
    let p0 = Plane::new(Vector3::unit_x(), 1.0f64);
    let p1 = Plane::new(Vector3::unit_y(), 2.0f64);
    let p2 = Plane::new(Vector3::unit_z(), 3.0f64);
    let point = p0.intersection(&(p1, p2));
    assert!(point.is_some());
    assert!(p0.intersects(&(p1, p2)));

    let point = point.unwrap();
    assert_ulps_eq!(point.x, &1.0f64);
    assert_ulps_eq!(point.y, &2.0f64);
    assert_ulps_eq!(point.z, &3.0f64);

    let p0 = Plane::new(Vector3::unit_y(), 1.0f64);
    let p1 = Plane::new(Vector3::unit_y(), 2.0f64);
    let p2 = Plane::new(Vector3::unit_z(), 3.0f64);
    let point = p0.intersection(&(p1, p2));
    assert!(point.is_none());
    assert!(!p0.intersects(&(p1, p2)));
}
