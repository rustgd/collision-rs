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

use std::fmt::Debug;

use cgmath::BaseNum;
use cgmath::prelude::*;

use {Aabb, MinMax};

/// An intersection test with a result.
///
/// An example would be a Ray vs AABB intersection test that returns a Point in space.
///
pub trait Continuous<RHS> {
    /// Result returned by the intersection test
    type Result;

    /// Intersection test
    fn intersection(&self, &RHS) -> Option<Self::Result>;
}

/// A boolean intersection test.
///
pub trait Discrete<RHS> {
    /// Intersection test
    fn intersects(&self, &RHS) -> bool;
}

/// Boolean containment test.
///
pub trait Contains<RHS> {
    /// Containment test
    #[inline]
    fn contains(&self, &RHS) -> bool;
}

/// Shape surface area
///
pub trait SurfaceArea {
    /// Result type returned from surface area computation
    type Scalar: BaseNum;

    /// Compute surface area
    fn surface_area(&self) -> Self::Scalar;
}

/// Build the union of two shapes.
///
pub trait Union<RHS = Self> {
    /// Union shape created
    type Output;

    /// Build the union shape of self and the given shape.
    fn union(&self, &RHS) -> Self::Output;
}

/// Primitive with axis aligned bounding box
pub trait HasAabb {
    /// Bounding box type
    type Aabb: Aabb + Clone + Union<Self::Aabb, Output = Self::Aabb> + Debug;

    /// Get the bounding box of the primitive in local space coordinates.
    fn get_bound(&self) -> Self::Aabb;
}

/// Minkowski support function for primitive
pub trait SupportFunction {
    /// Point type
    type Point: EuclideanSpace + MinMax;

    /// Get the support point on the shape in a given direction.
    ///
    /// ## Parameters
    ///
    /// - `direction`: The search direction in world space.
    /// - `transform`: The current local to world transform for this primitive.
    ///
    /// ## Returns
    ///
    /// Return the point that is furthest away from the origin, in the given search direction.
    /// For discrete shapes, the furthest vertex is enough, there is no need to do exact
    /// intersection point computation.
    ///
    /// ## Type parameters
    ///
    /// - `P`: Transform type
    fn support_point<T>(
        &self,
        direction: &<Self::Point as EuclideanSpace>::Diff,
        transform: &T,
    ) -> Self::Point
    where
        T: Transform<Self::Point>;
}

/// Discrete intersection test on transformed primitive
pub trait DiscreteTransformed<RHS> {
    /// Point type for transformation of self
    type Point: EuclideanSpace;

    /// Intersection test for transformed self
    fn intersects_transformed<T>(&self, _: &RHS, _: &T) -> bool
    where
        T: Transform<Self::Point>;
}

/// Continuous intersection test on transformed primitive
pub trait ContinuousTransformed<RHS> {
    /// Point type for transformation of self
    type Point: EuclideanSpace;

    /// Result of intersection test
    type Result: EuclideanSpace;

    /// Intersection test for transformed self
    fn intersection_transformed<T>(&self, _: &RHS, _: &T) -> Option<Self::Result>
    where
        T: Transform<Self::Point>;
}

/// Marker trait for a collision primitive.
pub trait Primitive
    : Debug + Clone + HasAabb + SupportFunction<Point = <<Self as HasAabb>::Aabb as Aabb>::Point>
    {
}

/// Implementation of marker trait for all types where the bounds are fulfilled
impl<T> Primitive for T
where
    T: Debug + Clone + HasAabb + SupportFunction<Point = <<Self as HasAabb>::Aabb as Aabb>::Point>,
{
}
