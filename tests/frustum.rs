use cgmath::{PerspectiveFov, Point3, Rad};
use collision::{Projection, Relation, Sphere};

#[test]
fn test_contains() {
    let frustum = PerspectiveFov {
        fovy: Rad(1f32),
        aspect: 1f32,
        near: 1f32,
        far: 10f32,
    }
    .to_frustum();
    assert_eq!(
        frustum.contains(&Sphere {
            center: Point3::new(0f32, 0f32, -5f32),
            radius: 1f32,
        }),
        Relation::In
    );
    assert_eq!(
        frustum.contains(&Sphere {
            center: Point3::new(0f32, 3f32, -5f32),
            radius: 1f32,
        }),
        Relation::Cross
    );
    assert_eq!(
        frustum.contains(&Sphere {
            center: Point3::new(0f32, 0f32, 5f32),
            radius: 1f32,
        }),
        Relation::Out
    );
}
