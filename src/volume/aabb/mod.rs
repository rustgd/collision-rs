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

pub use self::aabb2::Aabb2;
pub use self::aabb3::Aabb3;

use std::cmp::{Ordering, PartialOrd};

use cgmath::{BaseNum, Point2, Point3};
use cgmath::prelude::*;

mod aabb2;
mod aabb3;

pub(crate) fn min<S: PartialOrd + Copy>(lhs: S, rhs: S) -> S {
    match lhs.partial_cmp(&rhs) {
        Some(Ordering::Less) | Some(Ordering::Equal) | None => lhs,
        _ => rhs,
    }
}

pub(crate) fn max<S: PartialOrd + Copy>(lhs: S, rhs: S) -> S {
    match lhs.partial_cmp(&rhs) {
        Some(Ordering::Greater) | Some(Ordering::Equal) | None => lhs,
        _ => rhs,
    }
}

/// Compute the minimum/maximum of the given values
pub trait MinMax {
    /// Compute the minimum
    fn min(a: Self, b: Self) -> Self;

    /// Compute the maximum
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

/// Base trait describing an axis aligned bounding box.
pub trait Aabb: Sized {
    /// Scalar type
    type Scalar: BaseNum;

    /// Vector type
    type Diff: VectorSpace<Scalar = Self::Scalar> + ElementWise + Array<Element = Self::Scalar>;

    /// Point type
    type Point: EuclideanSpace<Scalar = Self::Scalar, Diff = Self::Diff> + MinMax;

    /// Create a new AABB using two points as opposing corners.
    fn new(p1: Self::Point, p2: Self::Point) -> Self;

    /// Create a new empty AABB
    fn zero() -> Self {
        let p = Self::Point::origin();
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
