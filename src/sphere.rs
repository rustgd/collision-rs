// Copyright 2013-2014 The CGMath Developers. For a full listing of the authors,
// refer to the Cargo.toml file at the top-level directory of this distribution.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//! Bounding sphere

use cgmath::{BaseFloat, Point3};
use cgmath::prelude::*;

use {Aabb3, Line3, Plane, Ray3};
use prelude::*;

/// Bounding sphere.
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct Sphere<S: BaseFloat> {
    /// Center point of the sphere in world space
    pub center: Point3<S>,
    /// Sphere radius
    pub radius: S,
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
        if d2 > s.radius * s.radius {
            return false;
        }
        return true;
    }
}

impl<S: BaseFloat> Discrete<Sphere<S>> for Sphere<S> {
    fn intersects(&self, s2: &Sphere<S>) -> bool {
        let s1 = self;

        let distance = s1.center.distance2(s2.center);
        let radiuses = s1.radius + s2.radius;

        distance <= radiuses
    }
}

impl<S: BaseFloat> Bound<S> for Sphere<S> {
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
        for c in aabb.to_corners().iter() {
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
            return self.clone();
        }
        if other.contains(self) {
            return other.clone();
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
            return self.clone();
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
