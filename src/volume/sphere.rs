//! Bounding sphere

use cgmath::prelude::*;
use cgmath::{BaseFloat, Point3, Vector3};

use crate::prelude::*;
use crate::{Aabb3, Line3, Plane, Ray3};

/// Bounding sphere.
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(
    feature = "serde",
    derive(Serialize, Deserialize),
    serde(crate = "serde_crate")
)]
pub struct Sphere<S: BaseFloat> {
    /// Center point of the sphere in world space
    pub center: Point3<S>,
    /// Sphere radius
    pub radius: S,
}

impl<S> Bound for Sphere<S>
where
    S: BaseFloat,
{
    type Point = Point3<S>;

    fn min_extent(&self) -> Point3<S> {
        self.center + Vector3::from_value(-self.radius)
    }

    fn max_extent(&self) -> Point3<S> {
        self.center + Vector3::from_value(self.radius)
    }

    fn with_margin(&self, add: Vector3<S>) -> Self {
        let max = add.x.max(add.y).max(add.z);
        Sphere {
            center: self.center,
            radius: self.radius + max,
        }
    }

    fn transform_volume<T>(&self, transform: &T) -> Self
    where
        T: Transform<Self::Point>,
    {
        Sphere {
            center: transform.transform_point(self.center),
            radius: self.radius,
        }
    }

    fn empty() -> Self {
        Self {
            center: Point3::origin(),
            radius: S::zero(),
        }
    }
}

impl<S: BaseFloat> Continuous<Ray3<S>> for Sphere<S> {
    type Result = Point3<S>;
    fn intersection(&self, r: &Ray3<S>) -> Option<Point3<S>> {
        let s = self;

        let l = s.center - r.origin;
        let tca = l.dot(r.direction);
        if tca < S::zero() {
            return None;
        }
        let d2 = l.dot(l) - tca * tca;
        if d2 > s.radius * s.radius {
            return None;
        }
        let thc = (s.radius * s.radius - d2).sqrt();
        Some(r.origin + r.direction * (tca - thc))
    }
}

impl<S: BaseFloat> Discrete<Ray3<S>> for Sphere<S> {
    fn intersects(&self, r: &Ray3<S>) -> bool {
        let s = self;
        let l = s.center - r.origin;
        let tca = l.dot(r.direction);
        if tca < S::zero() {
            return false;
        }
        let d2 = l.dot(l) - tca * tca;
        d2 <= s.radius * s.radius
    }
}

impl<S: BaseFloat> Discrete<Sphere<S>> for Sphere<S> {
    fn intersects(&self, s2: &Sphere<S>) -> bool {
        let s1 = self;

        let distance = s1.center.distance2(s2.center);
        let radiuses = s1.radius + s2.radius;

        distance <= radiuses * radiuses
    }
}

impl<S: BaseFloat> PlaneBound<S> for Sphere<S> {
    fn relate_plane(&self, plane: Plane<S>) -> Relation {
        let dist = self.center.dot(plane.n) - plane.d;
        if dist > self.radius {
            Relation::In
        } else if dist < -self.radius {
            Relation::Out
        } else {
            Relation::Cross
        }
    }
}

impl<S: BaseFloat> Contains<Aabb3<S>> for Sphere<S> {
    // will return true for border hits
    #[inline]
    fn contains(&self, aabb: &Aabb3<S>) -> bool {
        let radius_sq = self.radius * self.radius;
        for c in &aabb.to_corners() {
            if c.distance2(self.center) > radius_sq {
                return false;
            }
        }
        true
    }
}

impl<S: BaseFloat> Contains<Point3<S>> for Sphere<S> {
    #[inline]
    fn contains(&self, p: &Point3<S>) -> bool {
        self.center.distance2(*p) <= self.radius * self.radius
    }
}

impl<S: BaseFloat> Contains<Line3<S>> for Sphere<S> {
    #[inline]
    fn contains(&self, line: &Line3<S>) -> bool {
        self.contains(&line.origin) && self.contains(&line.dest)
    }
}

impl<S: BaseFloat> Contains<Sphere<S>> for Sphere<S> {
    #[inline]
    fn contains(&self, other: &Sphere<S>) -> bool {
        let center_dist = self.center.distance(other.center);
        (center_dist + other.radius) <= self.radius
    }
}

impl<S: BaseFloat> Union for Sphere<S> {
    type Output = Sphere<S>;

    fn union(&self, other: &Sphere<S>) -> Sphere<S> {
        if self.contains(other) {
            return *self;
        }
        if other.contains(self) {
            return *other;
        }
        let two = S::one() + S::one();
        let center_diff = other.center - self.center;
        let center_diff_s = center_diff.magnitude();
        let radius = (self.radius + other.radius + center_diff_s) / two;
        Sphere {
            radius,
            center: self.center + center_diff * (radius - self.radius) / center_diff_s,
        }
    }
}

impl<S: BaseFloat> Union<Aabb3<S>> for Sphere<S> {
    type Output = Sphere<S>;

    fn union(&self, aabb: &Aabb3<S>) -> Sphere<S> {
        if self.contains(aabb) {
            return *self;
        }
        let aabb_radius = aabb.max().distance(aabb.center());
        if aabb.contains(self) {
            return Sphere {
                center: aabb.center(),
                radius: aabb_radius,
            };
        }
        let two = S::one() + S::one();
        let center_diff = aabb.center() - self.center;
        let center_diff_s = aabb.center().distance(self.center);
        let radius = (self.radius + aabb_radius + center_diff_s) / two;
        Sphere {
            center: self.center + center_diff * (radius - self.radius) / center_diff_s,
            radius,
        }
    }
}

impl<S: BaseFloat> SurfaceArea for Sphere<S> {
    type Scalar = S;

    fn surface_area(&self) -> S {
        use std::f64::consts::PI;

        let two = S::one() + S::one();
        let four = two + two;
        let pi = S::from(PI).unwrap();
        four * pi * self.radius * self.radius
    }
}
