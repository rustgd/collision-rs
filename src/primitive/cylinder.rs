use cgmath::{BaseFloat, Point3, Vector3};
use cgmath::prelude::*;

use {Aabb3, Ray3};
use prelude::*;

/// Cylinder primitive
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
        result = result.normalize();
        if result.is_zero() {
            result = Zero::zero(); // cancel out any inconsistencies
        } else {
            result = result * self.radius;
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

        let four = two + two;
        let dr = b * b - four * a * c;
        if dr < S::zero() {
            return false;
        }
        let drsqrt = dr.sqrt();
        let t1 = (-b + drsqrt) / (two * a);
        let t2 = (-b - drsqrt) / (two * a);

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
        pc.y <= self.half_height && pc.y >= -self.half_height
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

        let pc = r.origin + r.direction * t;
        if (pc.y > self.half_height) || (pc.y < -self.half_height) {
            return None;
        }

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

        Some(r.origin + r.direction * t)
    }
}
