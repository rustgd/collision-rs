use cgmath::*;
use collision::*;

#[test]
fn test_intersection() {
    let sphere = Sphere {
        center: Point3::new(0f64, 0f64, 0f64),
        radius: 1f64,
    };
    let r0 = Ray::new(
        Point3::new(0f64, 0f64, 5f64),
        Vector3::new(0f64, 0f64, -5f64).normalize(),
    );
    let r1 = Ray::new(
        Point3::new(1f64.cos(), 0f64, 5f64),
        Vector3::new(0f64, 0f64, -5f64).normalize(),
    );
    let r2 = Ray::new(
        Point3::new(1f64, 0f64, 5f64),
        Vector3::new(0f64, 0f64, -5f64).normalize(),
    );
    let r3 = Ray::new(
        Point3::new(2f64, 0f64, 5f64),
        Vector3::new(0f64, 0f64, -5f64).normalize(),
    );
    assert_eq!(
        sphere.intersection(&r0),
        Some(Point3::new(0f64, 0f64, 1f64))
    );
    assert!(sphere.intersects(&r0));
    assert_ulps_eq!(
        sphere.intersection(&r1).unwrap(),
        &Point3::new(1f64.cos(), 0f64, 1f64.sin())
    );
    assert!(sphere.intersects(&r1));
    assert_eq!(
        sphere.intersection(&r2),
        Some(Point3::new(1f64, 0f64, 0f64))
    );
    assert!(sphere.intersects(&r2));
    assert_eq!(sphere.intersection(&r3), None);
    assert!(!sphere.intersects(&r3));
}

#[test]
fn test_bound() {
    let point = Point3::new(1f32, 2.0, 3.0);
    let sphere = Sphere {
        center: point,
        radius: 1.0,
    };
    let normal = vec3(0f32, 0.0, 1.0);

    assert_eq!(
        sphere.relate_plane(Plane::from_point_normal(point, normal)),
        Relation::Cross
    );
    assert_eq!(
        sphere.relate_plane(Plane::from_point_normal(point + normal * -3.0, normal)),
        Relation::In
    );
    assert_eq!(
        sphere.relate_plane(Plane::from_point_normal(point + normal * 3.0, normal)),
        Relation::Out
    );
}

#[test]
fn test_sphere_contains_point() {
    let sphere = Sphere {
        center: Point3::new(1f32, 2., 3.),
        radius: 1.,
    };

    let inside_p = Point3::new(1f32, 2.2, 3.);
    let outside_p = Point3::new(11f32, 2.2, 3.);

    assert!(sphere.contains(&inside_p));
    assert!(!sphere.contains(&outside_p));
}

#[test]
fn test_sphere_contains_line() {
    let sphere = Sphere {
        center: Point3::new(1f32, 2., 3.),
        radius: 1.,
    };

    let inside = Line3::new(Point3::new(1f32, 2., 3.), Point3::new(1f32, 2.2, 3.));
    let outside = Line3::new(Point3::new(11f32, 2., 3.), Point3::new(11f32, 2.2, 3.));
    let inside_out = Line3::new(Point3::new(1f32, 2., 3.), Point3::new(11f32, 2.2, 3.));

    assert!(sphere.contains(&inside));
    assert!(!sphere.contains(&outside));
    assert!(!sphere.contains(&inside_out));
}

#[test]
fn test_sphere_contains_sphere() {
    let sphere = Sphere {
        center: Point3::new(1f32, 2., 3.),
        radius: 1.,
    };

    let inside = Sphere {
        center: Point3::new(1f32, 2., 3.),
        radius: 0.1,
    };
    let outside = Sphere {
        center: Point3::new(11f32, 2., 3.),
        radius: 0.1,
    };
    let inside_out = Sphere {
        center: Point3::new(1f32, 2., 3.),
        radius: 10.,
    };

    assert!(sphere.contains(&inside));
    assert!(!sphere.contains(&outside));
    assert!(!sphere.contains(&inside_out));
}

#[test]
fn test_sphere_contains_aabb() {
    let sphere = Sphere {
        center: Point3::new(1f32, 2., 3.),
        radius: 1.,
    };

    let inside = Aabb3::new(Point3::new(1f32, 2., 3.), Point3::new(1f32, 2.2, 3.));
    let outside = Aabb3::new(Point3::new(11f32, 2., 3.), Point3::new(11f32, 2.2, 3.));
    let inside_out = Aabb3::new(Point3::new(1f32, 2., 3.), Point3::new(11f32, 2.2, 3.));
    let edge_case = Aabb3::new(Point3::new(1f32, 1.01, 3.), Point3::new(1.99f32, 2., 3.99));

    assert!(sphere.contains(&inside));
    assert!(!sphere.contains(&outside));
    assert!(!sphere.contains(&inside_out));
    assert!(!sphere.contains(&edge_case));
}

#[test]
fn test_sphere_union_sphere() {
    let base = Sphere {
        center: Point3::new(0., 0., 0.),
        radius: 5.,
    };

    let inside = Sphere {
        center: Point3::new(1., 1., 1.),
        radius: 1.,
    };
    let outside = Sphere {
        center: Point3::new(8., 8., 8.),
        radius: 1.,
    };
    let inside_out = Sphere {
        center: Point3::new(4., 4., 4.),
        radius: 3.,
    };

    assert_eq!(base, base.union(&inside));
    assert_eq!(
        Sphere {
            center: Point3::new(2.8452994616207485, 2.8452994616207485, 2.8452994616207485),
            radius: 9.928203230275509,
        },
        base.union(&outside)
    );
    assert_eq!(
        Sphere {
            center: Point3::new(1.4226497308103743, 1.4226497308103743, 1.4226497308103743),
            radius: 7.464101615137754,
        },
        base.union(&inside_out)
    );
}

#[test]
fn test_sphere_union_aabb3() {
    let base = Sphere {
        center: Point3::new(0., 0., 0.),
        radius: 5.,
    };

    let inside = Aabb3::new(Point3::new(0., 0., 0.), Point3::new(2., 2., 2.));
    let outside = Aabb3::new(Point3::new(6., 6., 6.), Point3::new(9., 9., 9.));
    let inside_out = Aabb3::new(Point3::new(4., 4., 4.), Point3::new(7., 7., 7.));

    assert_eq!(base, base.union(&inside));
    assert_eq!(
        Sphere {
            center: Point3::new(3.0566243270259355, 3.0566243270259355, 3.0566243270259355),
            radius: 10.294228634059948,
        },
        base.union(&outside)
    );
    assert_eq!(
        Sphere {
            center: Point3::new(2.0566243270259355, 2.0566243270259355, 2.0566243270259355),
            radius: 8.56217782649107,
        },
        base.union(&inside_out)
    );
}

#[test]
fn test_surface_area() {
    let base = Sphere {
        center: Point3::new(2., 1., -4.),
        radius: 2.,
    };

    // 4 * pi * r^2
    assert!((4. * std::f64::consts::PI * 2. * 2.) - base.surface_area() <= f64::EPSILON);
}
