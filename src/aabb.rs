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

//! Axis-aligned bounding boxes
//!
//! An AABB is a geometric object which encompasses a set of points and is not
//! rotated. It is either a rectangle or a rectangular prism (depending on the
//! dimension) where the slope of every line is either 0 or undefined. These
//! are useful for very cheap collision detection.

use std::fmt;
use std::cmp::{PartialOrd, Ordering};

use cgmath::{EuclideanSpace, Point2, Point3, One, Zero};
use cgmath::{VectorSpace, Array, Vector2, Vector3};
use cgmath::{BaseNum, BaseFloat, ElementWise};
use cgmath::Transform;

use {Ray2, Ray3, Plane, Sphere, Line2, Line3};
use bound::{Bound, Relation};
use intersect::{Continuous, Discrete, Contains};
use ops::Union;
use geometry::SurfaceArea;

fn min<S: PartialOrd + Copy>(lhs: S, rhs: S) -> S {
    match lhs.partial_cmp(&rhs) {
        Some(Ordering::Less) |
        Some(Ordering::Equal) |
        None => lhs,
        _ => rhs,
    }
}
fn max<S: PartialOrd + Copy>(lhs: S, rhs: S) -> S {
    match lhs.partial_cmp(&rhs) {
        Some(Ordering::Greater) |
        Some(Ordering::Equal) |
        None => lhs,
        _ => rhs,
    }
}

pub trait MinMax {
    fn min(a: Self, b: Self) -> Self;
    fn max(a: Self, b: Self) -> Self;
}

impl<S: PartialOrd> MinMax for Point2<S>
where
    S: BaseNum,
{
    fn min(a: Point2<S>, b: Point2<S>) -> Point2<S> {
        Point2::new(min(a.x, b.x), min(a.y, b.y))
    }

    fn max(a: Point2<S>, b: Point2<S>) -> Point2<S> {
        Point2::new(max(a.x, b.x), max(a.y, b.y))
    }
}

impl<S: PartialOrd> MinMax for Point3<S>
where
    S: BaseNum,
{
    fn min(a: Point3<S>, b: Point3<S>) -> Point3<S> {
        Point3::new(min(a.x, b.x), min(a.y, b.y), min(a.z, b.z))
    }

    fn max(a: Point3<S>, b: Point3<S>) -> Point3<S> {
        Point3::new(max(a.x, b.x), max(a.y, b.y), max(a.z, b.z))
    }
}

pub trait Aabb: Sized {
    type Scalar: BaseNum;
    type Diff: VectorSpace<Scalar = Self::Scalar> + ElementWise + Array<Element = Self::Scalar>;
    type Point: EuclideanSpace<Scalar = Self::Scalar, Diff = Self::Diff> + MinMax;

    /// Create a new AABB using two points as opposing corners.
    fn new(p1: Self::Point, p2: Self::Point) -> Self;

    /// Create a new empty AABB
    fn zero() -> Self {
        let p = Self::Point::from_value(Self::Scalar::zero());
        Self::new(p, p)
    }

    /// Return a shared reference to the point nearest to (-inf, -inf).
    fn min(&self) -> Self::Point;

    /// Return a shared reference to the point nearest to (inf, inf).
    fn max(&self) -> Self::Point;

    /// Return the dimensions of this AABB.
    #[inline]
    fn dim(&self) -> Self::Diff {
        self.max() - self.min()
    }

    /// Return the volume this AABB encloses.
    #[inline]
    fn volume(&self) -> Self::Scalar {
        self.dim().product()
    }

    /// Return the center point of this AABB.
    #[inline]
    fn center(&self) -> Self::Point {
        let two = Self::Scalar::one() + Self::Scalar::one();
        self.min() + self.dim() / two
    }

    /// Returns a new AABB that is grown to include the given point.
    fn grow(&self, p: Self::Point) -> Self {
        Aabb::new(MinMax::min(self.min(), p), MinMax::max(self.max(), p))
    }

    /// Add a vector to every point in the AABB, returning a new AABB.
    #[inline]
    fn add_v(&self, v: Self::Diff) -> Self {
        Aabb::new(self.min() + v, self.max() + v)
    }

    /// Add a margin of the given width around the AABB, returning a new AABB.
    #[inline]
    fn add_margin(&self, margin: Self::Diff) -> Self;

    /// Multiply every point in the AABB by a scalar, returning a new AABB.
    #[inline]
    fn mul_s(&self, s: Self::Scalar) -> Self {
        Aabb::new(self.min() * s, self.max() * s)
    }

    /// Multiply every point in the AABB by a vector, returning a new AABB.
    fn mul_v(&self, v: Self::Diff) -> Self {
        let min = Self::Point::from_vec(self.min().to_vec().mul_element_wise(v));
        let max = Self::Point::from_vec(self.max().to_vec().mul_element_wise(v));
        Aabb::new(min, max)
    }

    /// Apply an arbitrary transform to the corners of this bounding box,
    /// return a new conservative bound.
    fn transform<T>(&self, transform: &T) -> Self
    where
        T: Transform<Self::Point>;
}

/// A two-dimensional AABB, aka a rectangle.
#[derive(Copy, Clone, PartialEq)]
#[cfg_attr(feature = "eders", derive(Serialize, Deserialize))]
pub struct Aabb2<S> {
    pub min: Point2<S>,
    pub max: Point2<S>,
}

impl<S: BaseNum> Aabb2<S> {
    /// Construct a new axis-aligned bounding box from two points.
    #[inline]
    pub fn new(p1: Point2<S>, p2: Point2<S>) -> Aabb2<S> {
        Aabb2 {
            min: Point2::new(min(p1.x, p2.x), min(p1.y, p2.y)),
            max: Point2::new(max(p1.x, p2.x), max(p1.y, p2.y)),
        }
    }

    /// Compute corners.
    #[inline]
    pub fn to_corners(&self) -> [Point2<S>; 4] {
        [
            self.min,
            Point2::new(self.max.x, self.min.y),
            Point2::new(self.min.x, self.max.y),
            self.max,
        ]
    }
}

impl<S: BaseNum> Aabb for Aabb2<S> {
    type Scalar = S;
    type Diff = Vector2<S>;
    type Point = Point2<S>;

    #[inline]
    fn new(p1: Point2<S>, p2: Point2<S>) -> Aabb2<S> {
        Aabb2::new(p1, p2)
    }

    #[inline]
    fn min(&self) -> Point2<S> {
        self.min
    }

    #[inline]
    fn max(&self) -> Point2<S> {
        self.max
    }

    #[inline]
    fn add_margin(&self, margin: Self::Diff) -> Self {
        Aabb2::new(
            Point2::new(self.min.x - margin.x, self.min.y - margin.y),
            Point2::new(self.max.x + margin.x, self.max.y + margin.y),
        )
    }

    #[inline]
    fn transform<T>(&self, transform: &T) -> Self
    where
        T: Transform<Point2<S>>,
    {
        let corners = self.to_corners();
        let transformed_first = transform.transform_point(corners[0]);
        let base = Self::new(transformed_first, transformed_first);
        corners[1..].iter().fold(base, |u, &corner| {
            u.grow(transform.transform_point(corner))
        })
    }
}

impl<S: BaseNum> Contains<Point2<S>> for Aabb2<S> {
    #[inline]
    fn contains(&self, p: &Point2<S>) -> bool {
        let v_min = p - self.min();
        let v_max = self.max() - p;
        v_min.x >= S::zero() && v_min.y >= S::zero() && v_max.x > S::zero() && v_max.y > S::zero()
    }
}

impl<S: BaseNum> Contains<Aabb2<S>> for Aabb2<S> {
    #[inline]
    fn contains(&self, other: &Aabb2<S>) -> bool {
        let other_min = other.min();
        let other_max = other.max();

        other_min.x >= self.min.x && other_min.y >= self.min.y && other_max.x <= self.max.x &&
            other_max.y <= self.max.y
    }
}

impl<S: BaseNum> Contains<Line2<S>> for Aabb2<S> {
    #[inline]
    fn contains(&self, line: &Line2<S>) -> bool {
        self.contains(&line.origin) && self.contains(&line.dest)
    }
}

impl<S: BaseNum> Union for Aabb2<S> {
    type Output = Aabb2<S>;

    fn union(&self, other: &Aabb2<S>) -> Aabb2<S> {
        self.grow(other.min()).grow(other.max())
    }
}

impl<S: BaseNum> SurfaceArea<S> for Aabb2<S> {
    fn surface_area(&self) -> S {
        self.dim().x * self.dim().y
    }
}

impl<S: BaseNum> fmt::Debug for Aabb2<S> {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "[{:?} - {:?}]", self.min, self.max)
    }
}

/// A three-dimensional AABB, aka a rectangular prism.
#[derive(Copy, Clone, PartialEq)]
#[cfg_attr(feature = "eders", derive(Serialize, Deserialize))]
pub struct Aabb3<S> {
    pub min: Point3<S>,
    pub max: Point3<S>,
}

impl<S: BaseNum> Aabb3<S> {
    /// Construct a new axis-aligned bounding box from two points.
    #[inline]
    pub fn new(p1: Point3<S>, p2: Point3<S>) -> Aabb3<S> {
        Aabb3 {
            min: Point3::new(min(p1.x, p2.x), min(p1.y, p2.y), min(p1.z, p2.z)),
            max: Point3::new(max(p1.x, p2.x), max(p1.y, p2.y), max(p1.z, p2.z)),
        }
    }

    /// Compute corners.
    #[inline]
    pub fn to_corners(&self) -> [Point3<S>; 8] {
        [
            self.min,
            Point3::new(self.max.x, self.min.y, self.min.z),
            Point3::new(self.min.x, self.max.y, self.min.z),
            Point3::new(self.max.x, self.max.y, self.min.z),
            Point3::new(self.min.x, self.min.y, self.max.z),
            Point3::new(self.max.x, self.min.y, self.max.z),
            Point3::new(self.min.x, self.max.y, self.max.z),
            self.max,
        ]
    }
}

impl<S: BaseNum> Aabb for Aabb3<S> {
    type Scalar = S;
    type Diff = Vector3<S>;
    type Point = Point3<S>;

    #[inline]
    fn new(p1: Point3<S>, p2: Point3<S>) -> Aabb3<S> {
        Aabb3::new(p1, p2)
    }

    #[inline]
    fn min(&self) -> Point3<S> {
        self.min
    }

    #[inline]
    fn max(&self) -> Point3<S> {
        self.max
    }

    #[inline]
    fn add_margin(&self, margin: Self::Diff) -> Self {
        Aabb3::new(
            Point3::new(
                self.min.x - margin.x,
                self.min.y - margin.y,
                self.min.z - margin.z,
            ),
            Point3::new(
                self.max.x + margin.x,
                self.max.y + margin.y,
                self.max.z + margin.z,
            ),
        )
    }

    #[inline]
    fn transform<T>(&self, transform: &T) -> Self
    where
        T: Transform<Point3<S>>,
    {
        let corners = self.to_corners();
        let transformed_first = transform.transform_point(corners[0]);
        let base = Self::new(transformed_first, transformed_first);
        corners[1..].iter().fold(base, |u, &corner| {
            u.grow(transform.transform_point(corner))
        })
    }
}

impl<S: BaseNum> Contains<Point3<S>> for Aabb3<S> {
    #[inline]
    fn contains(&self, p: &Point3<S>) -> bool {
        let v_min = p - self.min();
        let v_max = self.max() - p;
        v_min.x >= S::zero() && v_min.y >= S::zero() && v_min.z >= S::zero() &&
            v_max.x > S::zero() && v_max.y > S::zero() && v_max.z > S::zero()
    }
}

impl<S: BaseNum> Contains<Aabb3<S>> for Aabb3<S> {
    #[inline]
    fn contains(&self, other: &Aabb3<S>) -> bool {
        let other_min = other.min();
        let other_max = other.max();

        other_min.x >= self.min.x && other_min.y >= self.min.y && other_min.z >= self.min.z &&
            other_max.x <= self.max.x &&
            other_max.y <= self.max.y && other_max.z <= self.max.z
    }
}

impl<S: BaseFloat> Contains<Sphere<S>> for Aabb3<S> {
    // will return true for border hits on both min and max extents
    #[inline]
    fn contains(&self, sphere: &Sphere<S>) -> bool {
        (sphere.center.x - sphere.radius) >= self.min.x &&
            (sphere.center.y - sphere.radius) >= self.min.y &&
            (sphere.center.z - sphere.radius) >= self.min.z &&
            (sphere.center.x + sphere.radius) <= self.max.x &&
            (sphere.center.y + sphere.radius) <= self.max.y &&
            (sphere.center.z + sphere.radius) <= self.max.z
    }
}

impl<S: BaseNum> Contains<Line3<S>> for Aabb3<S> {
    #[inline]
    fn contains(&self, line: &Line3<S>) -> bool {
        self.contains(&line.origin) && self.contains(&line.dest)
    }
}

impl<S: BaseNum> Union for Aabb3<S> {
    type Output = Aabb3<S>;

    fn union(&self, other: &Aabb3<S>) -> Aabb3<S> {
        self.grow(other.min()).grow(other.max())
    }
}

impl<S: BaseNum> fmt::Debug for Aabb3<S> {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "[{:?} - {:?}]", self.min, self.max)
    }
}

impl<S: BaseFloat> Continuous<Aabb2<S>> for Ray2<S> {
    type Result = Point2<S>;
    fn intersection(&self, aabb: &Aabb2<S>) -> Option<Point2<S>> {
        let ray = self;

        let mut tmin = S::neg_infinity();
        let mut tmax = S::infinity();

        if ray.direction.x != S::zero() {
            let tx1 = (aabb.min.x - ray.origin.x) / ray.direction.x;
            let tx2 = (aabb.max.x - ray.origin.x) / ray.direction.x;
            tmin = tmin.max(tx1.min(tx2));
            tmax = tmax.min(tx1.max(tx2));
        } else if ray.origin.x <= aabb.min.x || ray.origin.x >= aabb.max.x {
            return None;
        }

        if ray.direction.y != S::zero() {
            let ty1 = (aabb.min.y - ray.origin.y) / ray.direction.y;
            let ty2 = (aabb.max.y - ray.origin.y) / ray.direction.y;
            tmin = tmin.max(ty1.min(ty2));
            tmax = tmax.min(ty1.max(ty2));
        } else if ray.origin.y <= aabb.min.y || ray.origin.y >= aabb.max.y {
            return None;
        }

        if (tmin < S::zero() && tmax < S::zero()) || tmax < tmin {
            None
        } else {
            let t = if tmin >= S::zero() { tmin } else { tmax };
            Some(ray.origin + ray.direction * t)
        }
    }
}

impl<S: BaseFloat> Continuous<Ray2<S>> for Aabb2<S> {
    type Result = Point2<S>;

    fn intersection(&self, ray: &Ray2<S>) -> Option<Point2<S>> {
        ray.intersection(self)
    }
}

impl<S: BaseFloat> Discrete<Aabb2<S>> for Ray2<S> {
    fn intersects(&self, aabb: &Aabb2<S>) -> bool {
        let ray = self;

        let mut tmin = S::neg_infinity();
        let mut tmax = S::infinity();

        if ray.direction.x != S::zero() {
            let tx1 = (aabb.min.x - ray.origin.x) / ray.direction.x;
            let tx2 = (aabb.max.x - ray.origin.x) / ray.direction.x;
            tmin = tmin.max(tx1.min(tx2));
            tmax = tmax.min(tx1.max(tx2));
        } else if ray.origin.x <= aabb.min.x || ray.origin.x >= aabb.max.x {
            return false;
        }

        if ray.direction.y != S::zero() {
            let ty1 = (aabb.min.y - ray.origin.y) / ray.direction.y;
            let ty2 = (aabb.max.y - ray.origin.y) / ray.direction.y;
            tmin = tmin.max(ty1.min(ty2));
            tmax = tmax.min(ty1.max(ty2));
        } else if ray.origin.y <= aabb.min.y || ray.origin.y >= aabb.max.y {
            return false;
        }

        tmax >= tmin && (tmin >= S::zero() || tmax >= S::zero())
    }
}

impl<S: BaseFloat> Discrete<Ray2<S>> for Aabb2<S> {
    fn intersects(&self, ray: &Ray2<S>) -> bool {
        ray.intersects(&self)
    }
}

impl<S: BaseFloat> Continuous<Aabb3<S>> for Ray3<S> {
    type Result = Point3<S>;

    fn intersection(&self, aabb: &Aabb3<S>) -> Option<Point3<S>> {
        let ray = self;

        let inv_dir = Vector3::new(S::one(), S::one(), S::one()).div_element_wise(ray.direction);

        let mut t1 = (aabb.min.x - ray.origin.x) * inv_dir.x;
        let mut t2 = (aabb.max.x - ray.origin.x) * inv_dir.x;

        let mut tmin = t1.min(t2);
        let mut tmax = t1.max(t2);

        for i in 1..3 {
            t1 = (aabb.min[i] - ray.origin[i]) * inv_dir[i];
            t2 = (aabb.max[i] - ray.origin[i]) * inv_dir[i];

            tmin = tmin.max(t1.min(t2));
            tmax = tmax.min(t1.max(t2));
        }

        if (tmin < S::zero() && tmax < S::zero()) || tmax < tmin {
            None
        } else {
            let t = if tmin >= S::zero() { tmin } else { tmax };
            Some(ray.origin + ray.direction * t)
        }
    }
}

impl<S: BaseFloat> Continuous<Ray3<S>> for Aabb3<S> {
    type Result = Point3<S>;

    fn intersection(&self, ray: &Ray3<S>) -> Option<Point3<S>> {
        ray.intersection(self)
    }
}

impl<S: BaseFloat> Discrete<Aabb3<S>> for Ray3<S> {
    fn intersects(&self, aabb: &Aabb3<S>) -> bool {
        let ray = self;

        let inv_dir = Vector3::new(S::one(), S::one(), S::one()).div_element_wise(ray.direction);

        let mut t1 = (aabb.min.x - ray.origin.x) * inv_dir.x;
        let mut t2 = (aabb.max.x - ray.origin.x) * inv_dir.x;

        let mut tmin = t1.min(t2);
        let mut tmax = t1.max(t2);

        for i in 1..3 {
            t1 = (aabb.min[i] - ray.origin[i]) * inv_dir[i];
            t2 = (aabb.max[i] - ray.origin[i]) * inv_dir[i];

            tmin = tmin.max(t1.min(t2));
            tmax = tmax.min(t1.max(t2));
        }

        tmax >= tmin && (tmin >= S::zero() || tmax >= S::zero())
    }
}

impl<S: BaseFloat> Discrete<Ray3<S>> for Aabb3<S> {
    fn intersects(&self, ray: &Ray3<S>) -> bool {
        ray.intersects(self)
    }
}

impl<S: BaseFloat> Discrete<Aabb2<S>> for Aabb2<S> {
    fn intersects(&self, aabb: &Aabb2<S>) -> bool {
        let (a0, a1) = (self.min(), self.max());
        let (b0, b1) = (aabb.min(), aabb.max());

        a1.x > b0.x && a0.x < b1.x && a1.y > b0.y && a0.y < b1.y
    }
}

impl<S: BaseFloat> Discrete<Aabb3<S>> for Aabb3<S> {
    fn intersects(&self, aabb: &Aabb3<S>) -> bool {
        let (a0, a1) = (self.min(), self.max());
        let (b0, b1) = (aabb.min(), aabb.max());

        a1.x > b0.x && a0.x < b1.x && a1.y > b0.y && a0.y < b1.y && a1.z > b0.z && a0.z < b1.z
    }
}

impl<S: BaseFloat> Bound<S> for Aabb3<S> {
    fn relate_plane(&self, plane: Plane<S>) -> Relation {
        let corners = self.to_corners();
        let first = corners[0].relate_plane(plane);
        for p in corners[1..].iter() {
            if p.relate_plane(plane) != first {
                return Relation::Cross;
            }
        }
        first
    }
}

impl<S: BaseFloat> Union<Sphere<S>> for Aabb3<S> {
    type Output = Aabb3<S>;

    fn union(&self, sphere: &Sphere<S>) -> Aabb3<S> {
        self.grow(Point3::new(
            sphere.center.x - sphere.radius,
            sphere.center.y - sphere.radius,
            sphere.center.z - sphere.radius,
        )).grow(sphere.center + Vector3::from_value(sphere.radius))
    }
}

impl<S: BaseNum> SurfaceArea<S> for Aabb3<S> {
    fn surface_area(&self) -> S {
        let dim = self.dim();
        let two = S::one() + S::one();
        two * ((dim.x * dim.y) + (dim.x * dim.z) + (dim.y * dim.z))
    }
}
