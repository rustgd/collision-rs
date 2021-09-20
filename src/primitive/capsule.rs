use cgmath::prelude::*;
use cgmath::{BaseFloat, Point3, Vector3};

use crate::prelude::*;
use crate::primitive::util::cylinder_ray_quadratic_solve;
use crate::volume::Sphere;
use crate::{Aabb3, Ray3};

/// Capsule primitive
/// Capsule body is aligned with the Y axis, with local origin in the center of the capsule.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(
    feature = "serde",
    derive(Serialize, Deserialize),
    serde(crate = "serde_crate")
)]
pub struct Capsule<S> {
    half_height: S,
    radius: S,
}

impl<S> Capsule<S>
where
    S: BaseFloat,
{
    /// Create a new Capsule
    pub fn new(half_height: S, radius: S) -> Self {
        Self {
            half_height,
            radius,
        }
    }

    /// Get radius
    pub fn radius(&self) -> S {
        self.radius
    }

    /// Get height
    pub fn height(&self) -> S {
        self.half_height + self.half_height
    }
}

impl<S> Primitive for Capsule<S>
where
    S: BaseFloat,
{
    type Point = Point3<S>;

    fn support_point<T>(&self, direction: &Vector3<S>, transform: &T) -> Point3<S>
    where
        T: Transform<Point3<S>>,
    {
        let direction = transform.inverse_transform_vector(*direction).unwrap();

        let mut result = Point3::origin();
        result.y = direction.y.signum() * self.half_height;
        transform.transform_point(result + direction.normalize_to(self.radius))
    }
}

impl<S> ComputeBound<Aabb3<S>> for Capsule<S>
where
    S: BaseFloat,
{
    fn compute_bound(&self) -> Aabb3<S> {
        Aabb3::new(
            Point3::new(-self.radius, -self.half_height - self.radius, -self.radius),
            Point3::new(self.radius, self.half_height + self.radius, self.radius),
        )
    }
}

impl<S> ComputeBound<Sphere<S>> for Capsule<S>
where
    S: BaseFloat,
{
    fn compute_bound(&self) -> Sphere<S> {
        Sphere {
            center: Point3::origin(),
            radius: self.half_height + self.radius,
        }
    }
}

impl<S> Discrete<Ray3<S>> for Capsule<S>
where
    S: BaseFloat,
{
    fn intersects(&self, r: &Ray3<S>) -> bool {
        let (t1, t2) = match cylinder_ray_quadratic_solve(r, self.radius) {
            None => return false,
            Some(t) => t,
        };

        if t1 < S::zero() && t2 < S::zero() {
            return false;
        }

        let t = if t1 < S::zero() {
            t2
        } else if t2 < S::zero() {
            t1
        } else {
            t1.min(t2)
        };

        let pc = r.origin + r.direction * t;
        if pc.y <= self.half_height && pc.y >= -self.half_height {
            return true;
        }

        // top sphere
        let l = Vector3::new(-r.origin.x, self.half_height - r.origin.y, -r.origin.z);
        let tca = l.dot(r.direction);
        if tca > S::zero() {
            let d2 = l.dot(l) - tca * tca;
            if d2 <= self.radius * self.radius {
                return true;
            }
        }

        // bottom sphere
        let l = Vector3::new(-r.origin.x, -self.half_height - r.origin.y, -r.origin.z);
        let tca = l.dot(r.direction);
        if tca > S::zero() {
            let d2 = l.dot(l) - tca * tca;
            if d2 <= self.radius * self.radius {
                return true;
            }
        }

        false
    }
}

impl<S> Continuous<Ray3<S>> for Capsule<S>
where
    S: BaseFloat,
{
    type Result = Point3<S>;

    fn intersection(&self, r: &Ray3<S>) -> Option<Point3<S>> {
        let (t1, t2) = match cylinder_ray_quadratic_solve(r, self.radius) {
            None => return None,
            Some(t) => t,
        };

        if t1 < S::zero() && t2 < S::zero() {
            return None;
        }

        let mut t = if t1 < S::zero() {
            t2
        } else if t2 < S::zero() {
            t1
        } else {
            t1.min(t2)
        };

        // top sphere
        let l = Vector3::new(-r.origin.x, self.half_height - r.origin.y, -r.origin.z);
        let tca = l.dot(r.direction);
        if tca > S::zero() {
            let d2 = l.dot(l) - tca * tca;
            if d2 <= self.radius * self.radius {
                let thc = (self.radius * self.radius - d2).sqrt();
                let t0 = tca - thc;
                if t0 >= S::zero() && (t.is_nan() || t0 < t) {
                    t = t0;
                }
            }
        }

        // bottom sphere
        let l = Vector3::new(-r.origin.x, -self.half_height - r.origin.y, -r.origin.z);
        let tca = l.dot(r.direction);
        if tca > S::zero() {
            let d2 = l.dot(l) - tca * tca;
            if d2 <= self.radius * self.radius {
                let thc = (self.radius * self.radius - d2).sqrt();
                let t0 = tca - thc;
                if t0 >= S::zero() && (t.is_nan() || t0 < t) {
                    t = t0;
                }
            }
        }

        if t.is_nan() {
            return None;
        }

        let pc = r.origin + r.direction * t;
        let full_half_height = self.half_height + self.radius;
        if (pc.y > full_half_height) || (pc.y < -full_half_height) {
            None
        } else {
            Some(pc)
        }
    }
}

#[cfg(test)]
mod tests {
    use std;

    use cgmath::assert_ulps_eq;
    use cgmath::{Decomposed, Quaternion, Rad, Vector3};

    use super::*;

    #[test]
    fn test_capsule_aabb() {
        let capsule = Capsule::new(2., 1.);
        assert_eq!(
            Aabb3::new(Point3::new(-1., -3., -1.), Point3::new(1., 3., 1.)),
            capsule.compute_bound()
        );
    }

    #[test]
    fn test_capsule_support_1() {
        let capsule = Capsule::new(2., 1.);
        let direction = Vector3::new(1., 0., 0.);
        let transform = transform(0., 0., 0., 0.);
        let point = capsule.support_point(&direction, &transform);
        assert_ulps_eq!(Point3::new(1., 2., 0.), point);
    }

    #[test]
    fn test_capsule_support_2() {
        let capsule = Capsule::new(2., 1.);
        let direction = Vector3::new(0.5, -1., 0.).normalize();
        let transform = transform(0., 0., 0., 0.);
        let point = capsule.support_point(&direction, &transform);
        assert_ulps_eq!(Point3::new(0.447_213_65, -2.894_427_3, 0.), point);
    }

    #[test]
    fn test_capsule_support_3() {
        let capsule = Capsule::new(2., 1.);
        let direction = Vector3::new(0., 1., 0.);
        let transform = transform(0., 0., 0., 0.);
        let point = capsule.support_point(&direction, &transform);
        assert_ulps_eq!(Point3::new(0., 3., 0.), point);
    }

    #[test]
    fn test_capsule_support_4() {
        let capsule = Capsule::new(2., 1.);
        let direction = Vector3::new(1., 0., 0.);
        let transform = transform(10., 0., 0., 0.);
        let point = capsule.support_point(&direction, &transform);
        assert_ulps_eq!(Point3::new(11., 2., 0.), point);
    }

    #[test]
    fn test_capsule_support_5() {
        let capsule = Capsule::new(2., 1.);
        let direction = Vector3::new(1., 0., 0.);
        let transform = transform(0., 0., 0., std::f32::consts::PI);
        let point = capsule.support_point(&direction, &transform);
        assert_ulps_eq!(Point3::new(1., -2., 0.), point);
    }

    #[test]
    fn test_discrete_1() {
        let capsule = Capsule::new(2., 1.);
        let ray = Ray3::new(Point3::new(-3., 0., 0.), Vector3::new(1., 0., 0.));
        assert!(capsule.intersects(&ray));
    }

    #[test]
    fn test_discrete_2() {
        let capsule = Capsule::new(2., 1.);
        let ray = Ray3::new(Point3::new(-3., 0., 0.), Vector3::new(-1., 0., 0.));
        assert!(!capsule.intersects(&ray));
    }

    #[test]
    fn test_discrete_3() {
        let capsule = Capsule::new(2., 1.);
        let ray = Ray3::new(
            Point3::new(0., 3., 0.),
            Vector3::new(0.1, -1., 0.1).normalize(),
        );
        assert!(capsule.intersects(&ray));
    }

    #[test]
    fn test_discrete_4() {
        let capsule = Capsule::new(2., 1.);
        let ray = Ray3::new(
            Point3::new(0., 3., 0.),
            Vector3::new(0.1, 1., 0.1).normalize(),
        );
        assert!(!capsule.intersects(&ray));
    }

    #[test]
    fn test_discrete_5() {
        let capsule = Capsule::new(2., 1.);
        let ray = Ray3::new(
            Point3::new(0., 3., 0.),
            Vector3::new(0., -1., 0.).normalize(),
        );
        assert!(capsule.intersects(&ray));
    }

    #[test]
    fn test_continuous_1() {
        let capsule = Capsule::new(2., 1.);
        let ray = Ray3::new(Point3::new(-3., 0., 0.), Vector3::new(1., 0., 0.));
        assert_eq!(Some(Point3::new(-1., 0., 0.)), capsule.intersection(&ray));
    }

    #[test]
    fn test_continuous_2() {
        let capsule = Capsule::new(2., 1.);
        let ray = Ray3::new(Point3::new(-3., 0., 0.), Vector3::new(-1., 0., 0.));
        assert_eq!(None, capsule.intersection(&ray));
    }

    #[test]
    fn test_continuous_3() {
        let capsule = Capsule::new(2., 1.);
        let ray = Ray3::new(
            Point3::new(0., 4., 0.),
            Vector3::new(0.1, -1., 0.1).normalize(),
        );
        assert_eq!(
            Some(Point3::new(
                0.101_025_885_148_699_44,
                2.989_741_148_513_006,
                0.101_025_885_148_699_44
            )),
            capsule.intersection(&ray)
        );
    }

    #[test]
    fn test_continuous_4() {
        let capsule = Capsule::new(2., 1.);
        let ray = Ray3::new(
            Point3::new(0., 3., 0.),
            Vector3::new(0.1, 1., 0.1).normalize(),
        );
        assert_eq!(None, capsule.intersection(&ray));
    }

    #[test]
    fn test_continuous_5() {
        let capsule = Capsule::new(2., 1.);
        let ray = Ray3::new(
            Point3::new(0., 4., 0.),
            Vector3::new(0., -1., 0.).normalize(),
        );
        assert_eq!(Some(Point3::new(0., 3., 0.)), capsule.intersection(&ray));
    }

    // util
    fn transform(dx: f32, dy: f32, dz: f32, rot: f32) -> Decomposed<Vector3<f32>, Quaternion<f32>> {
        Decomposed {
            scale: 1.,
            rot: Quaternion::from_angle_z(Rad(rot)),
            disp: Vector3::new(dx, dy, dz),
        }
    }
}
