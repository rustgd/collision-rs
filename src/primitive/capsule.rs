use cgmath::{BaseFloat, Point3, Vector3};
use cgmath::prelude::*;

use {Aabb3, Ray3};
use prelude::*;

/// Capsule primitive
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "eders", derive(Serialize, Deserialize))]
pub struct Capsule<S> {
    half_height: S,
    radius: S,
}

impl<S> Capsule<S> {
    /// Create a new Capsule
    pub fn new(half_height: S, radius: S) -> Self {
        Self {
            half_height,
            radius,
        }
    }
}

impl<S> SupportFunction for Capsule<S>
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

        let mut result = Vector3::zero();
        if direction.y.is_sign_negative() {
            result[1] = -self.half_height;
        } else {
            result[1] = self.half_height;
        }
        transform.transform_point(Point3::from_vec(
            result + direction.normalize_to(self.radius),
        ))
    }
}

impl<S> HasAabb for Capsule<S>
where
    S: BaseFloat,
{
    type Aabb = Aabb3<S>;

    fn get_bound(&self) -> Self::Aabb {
        Aabb3::new(
            Point3::new(-self.radius, -self.half_height - self.radius, -self.radius),
            Point3::new(self.radius, self.half_height + self.radius, self.radius),
        )
    }
}

impl<S> Discrete<Ray3<S>> for Capsule<S>
where
    S: BaseFloat,
{
    fn intersects(&self, r: &Ray3<S>) -> bool {
        // cylinder
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
        // cylinder
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
        let full_half_height = self.half_height + self.radius;
        if (pc.y > full_half_height) || (pc.y < -full_half_height) {
            return None;
        }

        // top sphere
        let l = Vector3::new(-r.origin.x, self.half_height - r.origin.y, -r.origin.z);
        let tca = l.dot(r.direction);
        if tca > S::zero() {
            let d2 = l.dot(l) - tca * tca;
            if d2 <= self.radius * self.radius {
                let thc = (self.radius * self.radius - d2).sqrt();
                let t0 = tca - thc;
                if t0 >= S::zero() && t0 < t {
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
                if t0 >= S::zero() && t0 < t {
                    t = t0;
                }
            }
        }

        Some(r.origin + r.direction * t)
    }
}
