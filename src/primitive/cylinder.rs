use cgmath::{BaseFloat, Point3, Vector3};
use cgmath::prelude::*;

use {Aabb3, Ray3};
use prelude::*;

/// Cylinder primitive
/// Cylinder body is aligned with the Y axis, with local origin in the center of the cylinders.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "eders", derive(Serialize, Deserialize))]
pub struct Cylinder<S> {
    half_height: S,
    radius: S,
}

impl<S> Cylinder<S> {
    /// Create a new cylinder
    pub fn new(half_height: S, radius: S) -> Self {
        Self {
            half_height,
            radius,
        }
    }
}

impl<S> SupportFunction for Cylinder<S>
where
    S: BaseFloat,
{
    type Point = Point3<S>;

    fn support_point<T>(&self, direction: &Vector3<S>, transform: &T) -> Point3<S>
    where
        T: Transform<Point3<S>>,
    {
        let direction = transform
            .inverse_transform()
            .unwrap()
            .transform_vector(*direction);

        let mut result = direction;
        let negative = result.y.is_sign_negative();

        result.y = S::zero();
        if result.magnitude2().is_zero() {
            result = Zero::zero();
        } else {
            result = result.normalize();
            if result.is_zero() {
                result = Zero::zero(); // cancel out any inconsistencies
            } else {
                result = result * self.radius;
            }
        }
        if negative {
            result.y = -self.half_height;
        } else {
            result.y = self.half_height;
        }
        transform.transform_point(Point3::from_vec(result))
    }
}

impl<S> HasAabb for Cylinder<S>
where
    S: BaseFloat,
{
    type Aabb = Aabb3<S>;

    fn get_bound(&self) -> Self::Aabb {
        Aabb3::new(
            Point3::new(-self.radius, -self.half_height, -self.radius),
            Point3::new(self.radius, self.half_height, self.radius),
        )
    }
}

impl<S> Discrete<Ray3<S>> for Cylinder<S>
where
    S: BaseFloat,
{
    fn intersects(&self, r: &Ray3<S>) -> bool {
        let two = S::one() + S::one();

        let a = r.direction.x * r.direction.x + r.direction.z * r.direction.z;
        let b = two * (r.direction.x * r.origin.x + r.direction.z * r.origin.z);
        let c = r.origin.x * r.origin.x + r.origin.z * r.origin.z - self.radius * self.radius;

        // check if a is zero -> vector is parallel with y axis
        if a.is_zero() {
            if r.direction.y.is_zero() {
                return false;
            }

            return (r.origin.y >= -self.half_height && r.direction.y <= S::zero())
                || (r.origin.y <= self.half_height && r.direction.y >= S::zero());
        }
        let four = two + two;
        let dr = b * b - four * a * c;
        println!("{:?}", dr);
        if dr < S::zero() {
            return false;
        }
        let drsqrt = dr.sqrt();
        let t1 = (-b + drsqrt) / (two * a);
        let t2 = (-b - drsqrt) / (two * a);

        println!("{:?}", t1);
        println!("{:?}", t2);
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

        let n = -Vector3::unit_y();
        let tp = -(self.half_height + r.origin.dot(n)) / r.direction.dot(n);
        if tp >= S::zero() {
            let p = r.origin + r.direction * tp;
            if p.x * p.x + p.z * p.z < self.radius * self.radius {
                return true;
            }
        }

        let n = Vector3::unit_y();
        let tb = -(-self.half_height + r.origin.dot(n)) / r.direction.dot(n);
        if tb >= S::zero() {
            let p = r.origin + r.direction * tb;
            if p.x * p.x + p.z * p.z < self.radius * self.radius {
                return true;
            }
        }

        false
    }
}

impl<S> Continuous<Ray3<S>> for Cylinder<S>
where
    S: BaseFloat,
{
    type Result = Point3<S>;

    fn intersection(&self, r: &Ray3<S>) -> Option<Point3<S>> {
        let two = S::one() + S::one();

        let a = r.direction.x * r.direction.x + r.direction.z * r.direction.z;
        let b = two * (r.direction.x * r.origin.x + r.direction.z * r.origin.z);
        let c = r.origin.x * r.origin.x + r.origin.z * r.origin.z - self.radius * self.radius;

        if a.is_zero() {
            if r.direction.y.is_zero() {
                return None;
            }

            if r.origin.y >= self.half_height && r.direction.y < S::zero() {
                return Some(Point3::new(S::zero(), self.half_height, S::zero()));
            }
            if r.origin.y >= -self.half_height && r.direction.y < S::zero() {
                return Some(Point3::new(S::zero(), -self.half_height, S::zero()));
            }
            if r.origin.y <= -self.half_height && r.direction.y > S::zero() {
                return Some(Point3::new(S::zero(), -self.half_height, S::zero()));
            }
            if r.origin.y <= self.half_height && r.direction.y > S::zero() {
                return Some(Point3::new(S::zero(), self.half_height, S::zero()));
            }

            return None;
        }

        let four = two + two;
        let dr = b * b - four * a * c;
        if dr < S::zero() {
            return None;
        }
        let drsqrt = dr.sqrt();
        let t1 = (-b + drsqrt) / (two * a);
        let t2 = (-b - drsqrt) / (two * a);

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

        let n = -Vector3::unit_y();
        let tp = -(self.half_height + r.origin.dot(n)) / r.direction.dot(n);
        if tp >= S::zero() && tp < t {
            let p = r.origin + r.direction * tp;
            if p.x * p.x + p.z * p.z < self.radius * self.radius {
                t = tp;
            }
        }

        let n = Vector3::unit_y();
        let tb = -(-self.half_height + r.origin.dot(n)) / r.direction.dot(n);
        if tb >= S::zero() && tb < t {
            let p = r.origin + r.direction * tb;
            if p.x * p.x + p.z * p.z < self.radius * self.radius {
                t = tb;
            }
        }

        let pc = r.origin + r.direction * t;
        if (pc.y > self.half_height) || (pc.y < -self.half_height) {
            return None;
        }
        Some(r.origin + r.direction * t)
    }
}

#[cfg(test)]
mod tests {
    use std;

    use cgmath::{Decomposed, Quaternion, Rad, Vector3};

    use super::*;

    #[test]
    fn test_cylinder_aabb() {
        let cylinder = Cylinder::new(2., 1.);
        assert_eq!(
            Aabb3::new(Point3::new(-1., -2., -1.), Point3::new(1., 2., 1.)),
            cylinder.get_bound()
        );
    }

    #[test]
    fn test_cylinder_support_1() {
        let cylinder = Cylinder::new(2., 1.);
        let direction = Vector3::new(1., 0., 0.);
        let transform = transform(0., 0., 0., 0.);
        let point = cylinder.support_point(&direction, &transform);
        assert_ulps_eq!(Point3::new(1., 2., 0.), point);
    }

    #[test]
    fn test_cylinder_support_2() {
        let cylinder = Cylinder::new(2., 1.);
        let direction = Vector3::new(0.5, -1., 0.).normalize();
        let transform = transform(0., 0., 0., 0.);
        let point = cylinder.support_point(&direction, &transform);
        assert_ulps_eq!(Point3::new(1., -2., 0.), point);
    }

    #[test]
    fn test_cylinder_support_3() {
        let cylinder = Cylinder::new(2., 1.);
        let direction = Vector3::new(0., 1., 0.);
        let transform = transform(0., 0., 0., 0.);
        let point = cylinder.support_point(&direction, &transform);
        assert_ulps_eq!(Point3::new(0., 2., 0.), point);
    }

    #[test]
    fn test_cylinder_support_4() {
        let cylinder = Cylinder::new(2., 1.);
        let direction = Vector3::new(1., 0., 0.);
        let transform = transform(10., 0., 0., 0.);
        let point = cylinder.support_point(&direction, &transform);
        assert_ulps_eq!(Point3::new(11., 2., 0.), point);
    }

    #[test]
    fn test_cylinder_support_5() {
        let cylinder = Cylinder::new(2., 1.);
        let direction = Vector3::new(1., 0., 0.);
        let transform = transform(0., 0., 0., std::f32::consts::PI);
        let point = cylinder.support_point(&direction, &transform);
        assert_ulps_eq!(Point3::new(1., -2., 0.), point);
    }

    #[test]
    fn test_discrete_1() {
        let cylinder = Cylinder::new(2., 1.);
        let ray = Ray3::new(Point3::new(-3., 0., 0.), Vector3::new(1., 0., 0.));
        assert!(cylinder.intersects(&ray));
    }

    #[test]
    fn test_discrete_2() {
        let cylinder = Cylinder::new(2., 1.);
        let ray = Ray3::new(Point3::new(-3., 0., 0.), Vector3::new(-1., 0., 0.));
        assert!(!cylinder.intersects(&ray));
    }

    #[test]
    fn test_discrete_3() {
        let cylinder = Cylinder::new(2., 1.);
        let ray = Ray3::new(
            Point3::new(0., 3., 0.),
            Vector3::new(0.1, -1., 0.1).normalize(),
        );
        assert!(cylinder.intersects(&ray));
    }

    #[test]
    fn test_discrete_4() {
        let cylinder = Cylinder::new(2., 1.);
        let ray = Ray3::new(
            Point3::new(0., 3., 0.),
            Vector3::new(0.1, 1., 0.1).normalize(),
        );
        assert!(!cylinder.intersects(&ray));
    }

    #[test]
    fn test_continuous_1() {
        let cylinder = Cylinder::new(2., 1.);
        let ray = Ray3::new(Point3::new(-3., 0., 0.), Vector3::new(1., 0., 0.));
        assert_eq!(Some(Point3::new(-1., 0., 0.)), cylinder.intersection(&ray));
    }

    #[test]
    fn test_continuous_2() {
        let cylinder = Cylinder::new(2., 1.);
        let ray = Ray3::new(Point3::new(-3., 0., 0.), Vector3::new(-1., 0., 0.));
        assert_eq!(None, cylinder.intersection(&ray));
    }

    #[test]
    fn test_continuous_3() {
        let cylinder = Cylinder::new(2., 1.);
        let ray = Ray3::new(
            Point3::new(0., 3., 0.),
            Vector3::new(0.1, -1., 0.1).normalize(),
        );
        assert_eq!(Some(Point3::new(0.1, 2., 0.1)), cylinder.intersection(&ray));
    }

    #[test]
    fn test_continuous_4() {
        let cylinder = Cylinder::new(2., 1.);
        let ray = Ray3::new(
            Point3::new(0., 3., 0.),
            Vector3::new(0.1, 1., 0.1).normalize(),
        );
        assert_eq!(None, cylinder.intersection(&ray));
    }

    #[test]
    fn test_continuous_5() {
        let cylinder = Cylinder::new(2., 1.);
        let ray = Ray3::new(
            Point3::new(0., 3., 0.),
            Vector3::new(0., -1., 0.).normalize(),
        );
        assert_eq!(Some(Point3::new(0., 2., 0.)), cylinder.intersection(&ray));
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
