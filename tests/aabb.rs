use cgmath::InnerSpace;
use cgmath::{Point2, Point3};
use cgmath::{Vector2, Vector3};
use collision::{Aabb, Aabb2, Aabb3};
use collision::{Contains, Continuous, Discrete, SurfaceArea, Union};
use collision::{Line2, Line3, Ray2, Ray3, Sphere};
use collision::{Plane, PlaneBound, Ray, Relation};

#[test]
fn test_general() {
    let aabb = Aabb2::new(
        Point2::new(-20isize, 30isize),
        Point2::new(10isize, -10isize),
    );
    assert_eq!(aabb.min(), Point2::new(-20isize, -10isize));
    assert_eq!(aabb.max(), Point2::new(10isize, 30isize));
    assert_eq!(aabb.dim(), Vector2::new(30isize, 40isize));
    assert_eq!(aabb.volume(), 30isize * 40isize);
    assert_eq!(aabb.center(), Point2::new(-5isize, 10isize));

    assert!(aabb.contains(&Point2::new(0isize, 0isize)));
    assert!(!aabb.contains(&Point2::new(-50isize, -50isize)));
    assert!(!aabb.contains(&Point2::new(50isize, 50isize)));

    assert_eq!(aabb.grow(Point2::new(0isize, 0isize)), aabb);
    assert_eq!(
        aabb.grow(Point2::new(100isize, 100isize)),
        Aabb2::new(
            Point2::new(-20isize, -10isize),
            Point2::new(100isize, 100isize),
        )
    );
    assert_eq!(
        aabb.grow(Point2::new(-100isize, -100isize)),
        Aabb2::new(
            Point2::new(-100isize, -100isize),
            Point2::new(10isize, 30isize),
        )
    );

    assert_eq!(
        aabb.add_margin(Vector2::new(2isize, 2isize)),
        Aabb2::new(
            Point2::new(-22isize, -12isize),
            Point2::new(12isize, 32isize),
        )
    );

    let aabb = Aabb3::new(
        Point3::new(-20isize, 30isize, 5isize),
        Point3::new(10isize, -10isize, -5isize),
    );
    assert_eq!(aabb.min(), Point3::new(-20isize, -10isize, -5isize));
    assert_eq!(aabb.max(), Point3::new(10isize, 30isize, 5isize));
    assert_eq!(aabb.dim(), Vector3::new(30isize, 40isize, 10isize));
    assert_eq!(aabb.volume(), 30isize * 40isize * 10isize);
    assert_eq!(aabb.center(), Point3::new(-5isize, 10isize, 0isize));

    assert!(aabb.contains(&Point3::new(0isize, 0isize, 0isize)));
    assert!(!aabb.contains(&Point3::new(-100isize, 0isize, 0isize)));
    assert!(!aabb.contains(&Point3::new(100isize, 0isize, 0isize)));
    assert!(aabb.contains(&Point3::new(9isize, 29isize, -1isize)));
    assert!(!aabb.contains(&Point3::new(10isize, 30isize, 5isize)));
    assert!(aabb.contains(&Point3::new(-20isize, -10isize, -5isize)));
    assert!(!aabb.contains(&Point3::new(-21isize, -11isize, -6isize)));

    assert_eq!(
        aabb.add_v(Vector3::new(1isize, 2isize, 3isize)),
        Aabb3::new(
            Point3::new(-19isize, 32isize, 8isize),
            Point3::new(11isize, -8isize, -2isize),
        )
    );

    assert_eq!(
        aabb.mul_s(2isize),
        Aabb3::new(
            Point3::new(-40isize, -20isize, -10isize),
            Point3::new(20isize, 60isize, 10isize),
        )
    );

    assert_eq!(
        aabb.mul_v(Vector3::new(1isize, 2isize, 3isize)),
        Aabb3::new(
            Point3::new(-20isize, -20isize, -15isize),
            Point3::new(10isize, 60isize, 15isize),
        )
    );

    assert_eq!(
        aabb.add_margin(Vector3::new(2isize, 2isize, 2isize)),
        Aabb3::new(
            Point3::new(-22isize, -12isize, -7isize),
            Point3::new(12isize, 32isize, 7isize),
        )
    );
}

#[test]
fn test_ray2_intersect() {
    let aabb = Aabb2::new(Point2::new(-5.0f32, 5.0), Point2::new(5.0, 10.0));
    let ray1 = Ray::new(Point2::new(0.0f32, 0.0), Vector2::new(0.0, 1.0));
    let ray2 = Ray::new(Point2::new(-10.0f32, 0.0), Vector2::new(2.5, 1.0));
    let ray3 = Ray::new(Point2::new(0.0f32, 0.0), Vector2::new(-1.0, -1.0));
    let ray4 = Ray::new(Point2::new(3.0f32, 7.0), Vector2::new(1.0, 1.0));

    assert_eq!(ray1.intersection(&aabb), Some(Point2::new(0.0, 5.0)));
    assert!(ray1.intersects(&aabb));
    assert_eq!(ray2.intersection(&aabb), Some(Point2::new(2.5, 5.0)));
    assert!(ray2.intersects(&aabb));
    assert_eq!(ray3.intersection(&aabb), None);
    assert!(!ray3.intersects(&aabb));
    assert_eq!(ray4.intersection(&aabb), Some(Point2::new(5.0, 9.0)));
    assert!(ray4.intersects(&aabb));
}

#[test]
fn test_parallel_ray3_should_not_intersect() {
    let aabb = Aabb3::<f32>::new(Point3::new(1.0, 1.0, 1.0), Point3::new(5.0, 5.0, 5.0));
    let ray_x = Ray::new(Point3::new(0.0f32, 0.0, 0.0), Vector3::new(1.0, 0.0, 0.0));
    let ray_y = Ray::new(Point3::new(0.0f32, 0.0, 0.0), Vector3::new(0.0, 1.0, 0.0));
    let ray_z = Ray::new(Point3::new(0.0f32, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0));
    let ray_z_imprecise = Ray::new(
        Point3::new(0.0f32, 0.0, 0.0),
        Vector3::new(0.0001, 0.0001, 1.0),
    );

    assert_eq!(ray_x.intersection(&aabb), None);
    assert!(!ray_x.intersects(&aabb));
    assert_eq!(ray_y.intersection(&aabb), None);
    assert!(!ray_y.intersects(&aabb));
    assert_eq!(ray_z.intersection(&aabb), None);
    assert!(!ray_z.intersects(&aabb));
    assert_eq!(ray_z_imprecise.intersection(&aabb), None);
    assert!(!ray_z_imprecise.intersects(&aabb));
}

#[test]
fn test_oblique_ray3_should_intersect() {
    let aabb = Aabb3::<f32>::new(Point3::new(1.0, 1.0, 1.0), Point3::new(5.0, 5.0, 5.0));
    let ray1 = Ray::new(
        Point3::new(0.0f32, 0.0, 0.0),
        Vector3::new(1.0, 1.0, 1.0).normalize(),
    );
    let ray2 = Ray::new(Point3::new(0.0f32, 6.0, 0.0), Vector3::new(1.0, -1.0, 1.0));

    assert_eq!(ray1.intersection(&aabb), Some(Point3::new(1.0, 1.0, 1.0)));
    assert!(ray1.intersects(&aabb));
    assert_eq!(ray2.intersection(&aabb), Some(Point3::new(1.0, 5.0, 1.0)));
    assert!(ray2.intersects(&aabb));
}

#[test]
fn test_pointing_to_other_dir_ray3_should_not_intersect() {
    let aabb = Aabb3::<f32>::new(Point3::new(1.0, 1.0, 1.0), Point3::new(5.0, 5.0, 5.0));
    let ray_x = Ray::new(
        Point3::new(0.0f32, 2.0, 2.0),
        Vector3::new(-1.0, 0.01, 0.01),
    );
    let ray_y = Ray::new(
        Point3::new(2.0f32, 0.0, 2.0),
        Vector3::new(0.01, -1.0, 0.01),
    );
    let ray_z = Ray::new(
        Point3::new(2.0f32, 2.0, 0.0),
        Vector3::new(0.01, 0.01, -1.0),
    );

    let ray_x2 = Ray::new(Point3::new(6.0f32, 2.0, 2.0), Vector3::new(1.0, 0.0, 0.0));
    let ray_y2 = Ray::new(Point3::new(2.0f32, 6.0, 2.0), Vector3::new(0.0, 1.0, 0.0));
    let ray_z2 = Ray::new(Point3::new(2.0f32, 2.0, 6.0), Vector3::new(0.0, 0.0, 1.0));

    assert_eq!(ray_x.intersection(&aabb), None);
    assert!(!ray_x.intersects(&aabb));
    assert_eq!(ray_y.intersection(&aabb), None);
    assert!(!ray_y.intersects(&aabb));
    assert_eq!(ray_z.intersection(&aabb), None);
    assert!(!ray_z.intersects(&aabb));

    assert_eq!(ray_x2.intersection(&aabb), None);
    assert!(!ray_x2.intersects(&aabb));
    assert_eq!(ray_y2.intersection(&aabb), None);
    assert!(!ray_y2.intersects(&aabb));
    assert_eq!(ray_z2.intersection(&aabb), None);
    assert!(!ray_z2.intersects(&aabb));
}

#[test]
fn test_pointing_to_box_dir_ray3_should_intersect() {
    let aabb = Aabb3::<f32>::new(Point3::new(1.0, 1.0, 1.0), Point3::new(5.0, 5.0, 5.0));
    let ray_x = Ray::new(Point3::new(0.0f32, 2.0, 2.0), Vector3::new(1.0, 0.0, 0.0));
    let ray_y = Ray::new(Point3::new(2.0f32, 0.0, 2.0), Vector3::new(0.0, 1.0, 0.0));
    let ray_z = Ray::new(Point3::new(2.0f32, 2.0, 0.0), Vector3::new(0.0, 0.0, 1.0));

    let ray_x2 = Ray::new(Point3::new(6.0f32, 2.0, 2.0), Vector3::new(-1.0, 0.0, 0.0));
    let ray_y2 = Ray::new(Point3::new(2.0f32, 6.0, 2.0), Vector3::new(0.0, -1.0, 0.0));
    let ray_z2 = Ray::new(Point3::new(2.0f32, 2.0, 6.0), Vector3::new(0.0, 0.0, -1.0));

    assert_eq!(ray_x.intersection(&aabb), Some(Point3::new(1.0, 2.0, 2.0)));
    assert!(ray_x.intersects(&aabb));
    assert_eq!(ray_y.intersection(&aabb), Some(Point3::new(2.0, 1.0, 2.0)));
    assert!(ray_y.intersects(&aabb));
    assert_eq!(ray_z.intersection(&aabb), Some(Point3::new(2.0, 2.0, 1.0)));
    assert!(ray_z.intersects(&aabb));

    assert_eq!(ray_x2.intersection(&aabb), Some(Point3::new(5.0, 2.0, 2.0)));
    assert!(ray_x2.intersects(&aabb));
    assert_eq!(ray_y2.intersection(&aabb), Some(Point3::new(2.0, 5.0, 2.0)));
    assert!(ray_y2.intersects(&aabb));
    assert_eq!(ray_z2.intersection(&aabb), Some(Point3::new(2.0, 2.0, 5.0)));
    assert!(ray_z2.intersects(&aabb));
}

#[test]
fn test_corners() {
    let corners = Aabb2::new(Point2::new(-5.0f32, 5.0), Point2::new(5.0, 10.0)).to_corners();
    assert!(corners.contains(&Point2::new(-5f32, 10.0)));
    assert!(corners.contains(&Point2::new(5f32, 5.0)));

    let corners = Aabb3::new(
        Point3::new(-20isize, 30isize, 5isize),
        Point3::new(10isize, -10isize, -5isize),
    )
    .to_corners();
    assert!(corners.contains(&Point3::new(-20isize, 30isize, -5isize)));
    assert!(corners.contains(&Point3::new(10isize, 30isize, 5isize)));
    assert!(corners.contains(&Point3::new(10isize, -10isize, 5isize)));
}

#[test]
fn test_bound() {
    let aabb = Aabb3::new(Point3::new(-5.0f32, 5.0, 0.0), Point3::new(5.0, 10.0, 1.0));
    let plane1 =
        Plane::from_point_normal(Point3::new(0f32, 0.0, 0.0), Vector3::new(0f32, 0.0, 1.0));
    let plane2 =
        Plane::from_point_normal(Point3::new(-5.0f32, 4.0, 0.0), Vector3::new(0f32, 1.0, 0.0));
    let plane3 =
        Plane::from_point_normal(Point3::new(6.0f32, 0.0, 0.0), Vector3::new(1f32, 0.0, 0.0));
    assert_eq!(aabb.relate_plane(plane1), Relation::Cross);
    assert_eq!(aabb.relate_plane(plane2), Relation::In);
    assert_eq!(aabb.relate_plane(plane3), Relation::Out);
}

#[test]
fn test_aab3_should_not_intersect() {
    use collision::Discrete;
    let a = Aabb3::new(Point3::new(0., 0., 0.), Point3::new(10., 10., 10.));
    let b = Aabb3::new(Point3::new(15., 15., 15.), Point3::new(25., 25., 25.));
    assert!(!a.intersects(&b));
}

#[test]
fn test_aab3_should_intersect() {
    use collision::Discrete;
    let a = Aabb3::new(Point3::new(0., 0., 0.), Point3::new(10., 10., 10.));
    let b = Aabb3::new(Point3::new(5., 5., 5.), Point3::new(15., 15., 15.));
    assert!(a.intersects(&b));
}

#[test]
fn test_aabb3_should_not_intersect_overlap_x() {
    use collision::Discrete;
    let a = Aabb3::new(Point3::new(0., 0., 0.), Point3::new(10., 10., 10.));
    let b = Aabb3::new(Point3::new(5., 11., 11.), Point3::new(15., 13., 13.));
    assert!(!a.intersects(&b));
}

#[test]
fn test_aabb3_should_not_intersect_overlap_y() {
    use collision::Discrete;
    let a = Aabb3::new(Point3::new(0., 0., 0.), Point3::new(10., 10., 10.));
    let b = Aabb3::new(Point3::new(11., 5., 11.), Point3::new(15., 13., 13.));
    assert!(!a.intersects(&b));
}

#[test]
fn test_aabb3_should_not_intersect_overlap_z() {
    use collision::Discrete;
    let a = Aabb3::new(Point3::new(0., 0., 0.), Point3::new(10., 10., 10.));
    let b = Aabb3::new(Point3::new(11., 11., 5.), Point3::new(15., 13., 13.));
    assert!(!a.intersects(&b));
}

#[test]
fn test_aabb2_contains_point2() {
    let aabb = Aabb2::new(Point2::new(0., 0.), Point2::new(10., 10.));

    let inside = Point2::new(2., 2.);
    let outside = Point2::new(11., 11.);

    assert!(aabb.contains(&inside));
    assert!(!aabb.contains(&outside));

    let aabb = Aabb2::<usize>::new(Point2::new(0, 0), Point2::new(10, 10));

    let inside = Point2::new(2, 2);
    let outside = Point2::new(11, 11);

    assert!(aabb.contains(&inside));
    assert!(!aabb.contains(&outside));
}

#[test]
fn test_aabb2_contains_line2() {
    let aabb = Aabb2::new(Point2::new(0., 0.), Point2::new(10., 10.));

    let inside = Line2::new(Point2::new(1., 1.), Point2::new(2., 2.));
    let inside_out = Line2::new(Point2::new(1., 1.), Point2::new(12., 12.));
    let outside = Line2::new(Point2::new(11., 11.), Point2::new(12., 12.));

    assert!(aabb.contains(&inside));
    assert!(!aabb.contains(&outside));
    assert!(!aabb.contains(&inside_out));
}

#[test]
fn test_aabb2_contains_aabb2() {
    let aabb = Aabb2::new(Point2::new(0., 0.), Point2::new(10., 10.));

    let inside = Aabb2::new(Point2::new(1., 1.), Point2::new(2., 2.));
    let inside_out = Aabb2::new(Point2::new(1., 1.), Point2::new(12., 12.));
    let outside = Aabb2::new(Point2::new(11., 11.), Point2::new(12., 12.));

    assert!(aabb.contains(&inside));
    assert!(!aabb.contains(&outside));
    assert!(!aabb.contains(&inside_out));
}

#[test]
fn test_aabb3_contains_point3() {
    let aabb = Aabb3::new(Point3::new(0., 0., 0.), Point3::new(10., 10., 10.));

    let inside = Point3::new(3., 3., 3.);
    let outside = Point3::new(11., 11., 11.);

    assert!(aabb.contains(&inside));
    assert!(!aabb.contains(&outside));

    let aabb = Aabb3::<usize>::new(Point3::new(0, 0, 0), Point3::new(10, 10, 10));

    let inside = Point3::new(3, 3, 3);
    let outside = Point3::new(11, 11, 11);

    assert!(aabb.contains(&inside));
    assert!(!aabb.contains(&outside));
}

#[test]
fn test_aabb3_contains_line3() {
    let aabb = Aabb3::new(Point3::new(0., 0., 0.), Point3::new(10., 10., 10.));

    let inside = Line3::new(Point3::new(1., 1., 1.), Point3::new(2., 2., 2.));
    let inside_out = Line3::new(Point3::new(1., 1., 1.), Point3::new(12., 12., 12.));
    let outside = Line3::new(Point3::new(11., 11., 11.), Point3::new(12., 12., 12.));

    assert!(aabb.contains(&inside));
    assert!(!aabb.contains(&outside));
    assert!(!aabb.contains(&inside_out));
}

#[test]
fn test_aabb3_contains_aabb3() {
    let aabb = Aabb3::new(Point3::new(0., 0., 0.), Point3::new(10., 10., 10.));

    let inside = Aabb3::new(Point3::new(1., 1., 1.), Point3::new(2., 2., 2.));
    let inside_out = Aabb3::new(Point3::new(1., 1., 1.), Point3::new(12., 12., 12.));
    let outside = Aabb3::new(Point3::new(11., 11., 11.), Point3::new(12., 12., 12.));

    assert!(aabb.contains(&inside));
    assert!(!aabb.contains(&outside));
    assert!(!aabb.contains(&inside_out));
}

#[test]
fn test_aabb3_contains_sphere() {
    let aabb = Aabb3::new(Point3::new(0., 0., 0.), Point3::new(10., 10., 10.));

    let inside = Sphere {
        center: Point3::new(5., 5., 5.),
        radius: 1.,
    };

    let inside_out = Sphere {
        center: Point3::new(5., 5., 5.),
        radius: 10.,
    };
    let outside = Sphere {
        center: Point3::new(20., 20., 20.),
        radius: 1.,
    };

    assert!(aabb.contains(&inside));
    assert!(!aabb.contains(&outside));
    assert!(!aabb.contains(&inside_out));
}

#[test]
fn test_ray_aabb2_parallel() {
    let aabb = Aabb2::new(Point2::new(5., 5.), Point2::new(10., 10.));

    let ray = Ray2::new(Point2::new(2., 0.), Vector2::new(0., 1.));
    assert!(!ray.intersects(&aabb));
    assert!(ray.intersection(&aabb).is_none());

    let ray = Ray2::new(Point2::new(0., 2.), Vector2::new(1., 0.));
    assert!(!ray.intersects(&aabb));
    assert!(ray.intersection(&aabb).is_none());
}

#[test]
fn test_ray_aabb3_parallel() {
    let aabb = Aabb3::new(Point3::new(5., 5., 5.), Point3::new(10., 10., 10.));

    let ray = Ray3::new(Point3::new(2., 0., 2.), Vector3::new(0., 1., 0.));
    assert!(!ray.intersects(&aabb));
    assert!(ray.intersection(&aabb).is_none());

    let ray = Ray3::new(Point3::new(0., 2., 2.), Vector3::new(1., 0., 0.));
    assert!(!ray.intersects(&aabb));
    assert!(ray.intersection(&aabb).is_none());

    let ray = Ray3::new(Point3::new(2., 2., 0.), Vector3::new(0., 0., 1.));
    assert!(!ray.intersects(&aabb));
    assert!(ray.intersection(&aabb).is_none());
}

#[test]
fn test_aabb2_union_aabb2() {
    let base = Aabb2::new(Point2::new(0., 0.), Point2::new(10., 10.));

    let inside = Aabb2::new(Point2::new(2., 2.), Point2::new(5., 5.));
    let outside = Aabb2::new(Point2::new(12., 12.), Point2::new(15., 15.));
    let inside_out = Aabb2::new(Point2::new(2., 2.), Point2::new(12., 12.));

    assert_eq!(base, base.union(&inside));
    assert_eq!(
        Aabb2::new(Point2::new(0., 0.), Point2::new(15., 15.)),
        base.union(&outside)
    );
    assert_eq!(
        Aabb2::new(Point2::new(0., 0.), Point2::new(12., 12.)),
        base.union(&inside_out)
    );
}

#[test]
fn test_aabb3_union_aabb3() {
    let base = Aabb3::new(Point3::new(0., 0., 0.), Point3::new(10., 10., 10.));

    let inside = Aabb3::new(Point3::new(2., 2., 2.), Point3::new(5., 5., 5.));
    let outside = Aabb3::new(Point3::new(12., 12., 12.), Point3::new(15., 15., 15.));
    let inside_out = Aabb3::new(Point3::new(2., 2., 2.), Point3::new(12., 12., 12.));

    assert_eq!(base, base.union(&inside));
    assert_eq!(
        Aabb3::new(Point3::new(0., 0., 0.), Point3::new(15., 15., 15.)),
        base.union(&outside)
    );
    assert_eq!(
        Aabb3::new(Point3::new(0., 0., 0.), Point3::new(12., 12., 12.)),
        base.union(&inside_out)
    );
}

#[test]
fn test_aabb3_union_sphere() {
    let base = Aabb3::new(Point3::new(0., 0., 0.), Point3::new(10., 10., 10.));

    let inside = Sphere {
        center: Point3::new(5., 5., 5.),
        radius: 1.,
    };
    let outside = Sphere {
        center: Point3::new(14., 14., 14.),
        radius: 1.,
    };
    let inside_out = Sphere {
        center: Point3::new(8., 8., 8.),
        radius: 4.,
    };

    assert_eq!(base, base.union(&inside));
    assert_eq!(
        Aabb3::new(Point3::new(0., 0., 0.), Point3::new(15., 15., 15.)),
        base.union(&outside)
    );
    assert_eq!(
        Aabb3::new(Point3::new(0., 0., 0.), Point3::new(12., 12., 12.)),
        base.union(&inside_out)
    );
}

#[test]
fn test_aabb2_surface_area() {
    let aabb = Aabb2::new(Point2::new(0., 0.), Point2::new(10., 20.));
    assert!(200. - aabb.surface_area() <= f32::EPSILON);
}

#[test]
fn test_aabb3_surface_area() {
    let aabb = Aabb3::new(Point3::new(0., 0., 0.), Point3::new(10., 20., 30.));
    // 2 * (x*y + x*z + y*z)
    assert!(2200. - aabb.surface_area() <= f32::EPSILON);
}

#[test]
fn test_aabb2_transform() {
    let aabb = Aabb2::new(Point2::new(0., 0.), Point2::new(10., 20.));
    let transform = cgmath::Decomposed::<Vector2<f32>, cgmath::Basis2<f32>> {
        disp: Vector2::new(5., 3.),
        scale: 1.,
        rot: cgmath::Rotation2::from_angle(cgmath::Rad(0.)),
    };
    assert_eq!(
        Aabb2::new(Point2::new(5., 3.), Point2::new(15., 23.)),
        aabb.transform(&transform)
    );
}

#[test]
fn test_aabb3_transform() {
    let aabb = Aabb3::new(Point3::new(0., 0., 0.), Point3::new(10., 20., 30.));
    let transform = cgmath::Decomposed::<Vector3<f32>, cgmath::Basis3<f32>> {
        disp: Vector3::new(5., 3., 1.),
        scale: 1.,
        rot: cgmath::Rotation3::from_angle_z(cgmath::Rad(0.)),
    };
    assert_eq!(
        Aabb3::new(Point3::new(5., 3., 1.), Point3::new(15., 23., 31.)),
        aabb.transform(&transform)
    );
}
