// Copyright 2013 The CGMath Developers. For a full listing of the authors,
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

use {Ray2, Ray3, Plane, Line2};
use cgmath::{BaseFloat, Zero, EuclideanSpace};
use cgmath::{Point2, Point3};
use cgmath::{InnerSpace, Vector2};

pub trait Intersect<Result> {
    fn intersection(&self) -> Result;
}


impl<S: BaseFloat> Intersect<Option<Point3<S>>> for (Plane<S>, Ray3<S>) {
    fn intersection(&self) -> Option<Point3<S>> {
        let (ref p, ref r) = *self;

        let t = -(p.d + r.origin.dot(p.n)) / r.direction.dot(p.n);
        if t < Zero::zero() { None }
        else { Some(r.origin + r.direction * t) }
    }
}

/// See _Real-Time Collision Detection_, p. 210
impl<S: BaseFloat> Intersect<Option<Ray3<S>>> for (Plane<S>, Plane<S>) {
    fn intersection(&self) -> Option<Ray3<S>> {
        let (p1, p2) = *self;
        let d = p1.n.cross(p2.n);
        let denom = d.dot(d);
        if ulps_eq!(denom, &S::zero()) {
            None
        }
        else {
            let p = (p2.n * p1.d - p1.n * p2.d).cross(d) / denom;
            Some(Ray3::new(Point3::from_vec(p), d))
        }
    }
}

/// See _Real-Time Collision Detection_, p. 212 - 214
impl<S: BaseFloat> Intersect<Option<Point3<S>>> for (Plane<S>, Plane<S>, Plane<S>) {
    fn intersection(&self) -> Option<Point3<S>> {
        let (p1, p2, p3) = *self;
        let u = p2.n.cross(p3.n);
        let denom = p1.n.dot(u);
        if ulps_eq!(denom.abs(), &S::zero()) {
            None
        }
        else {
            let p = (u * p1.d + p1.n.cross(p2.n * p3.d - p3.n * p2.d)) / denom;
            Some(Point3::from_vec(p))
        }
    }
}

/// Determines if an intersection between a ray and a line segment is found.
impl<S: BaseFloat> Intersect<Option<Point2<S>>> for (Ray2<S>, Line2<S>) {
    fn intersection(&self) -> Option<Point2<S>> {
        let (ref ray, ref line) = *self;

        let p = ray.origin;
        let q = line.origin;
        let r = ray.direction;
        let s = Vector2::new(line.dest.x - line.origin.x, line.dest.y - line.origin.y);

        let cross_1 = r.perp_dot(s);
        let qmp = Vector2::new(q.x - p.x, q.y - p.y);
        let cross_2 = qmp.perp_dot(r);

        if cross_1 == S::zero() {
            if cross_2 != S::zero() {
                // parallel
                return None;
            }

            // collinear
            let q2mp = Vector2::new(line.dest.x - p.x, line.dest.y - p.y);
            let dot_1 = qmp.dot(r);
            let dot_2 = q2mp.dot(r);
            if (dot_1 <= S::zero() && dot_2 >= S::zero()) || (dot_1 >= S::zero() && dot_2 <= S::zero()) {
                return Some(p);
            }
            else if dot_1 >= S::zero() && dot_2 >= S::zero() {
                if dot_1 <= dot_2 {
                    return Some(q);
                }
                else {
                    return Some(line.dest);
                }
            }

            // no overlap exists
            return None;
        }

        let t = qmp.perp_dot(s) / cross_1;
        let u = cross_2 / cross_1;

        if S::zero() <= t && u >= S::zero() && u <= S::one() {
            return Some(Point2::new(p.x + t*r.x, p.y + t*r.y));
        }

        return None;
    }
}
