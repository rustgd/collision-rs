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

#![crate_type = "rlib"]
#![crate_type = "dylib"]

//! Computer graphics-centric math.
//!
//! This crate provides useful mathematical primitives and operations on them.
//! It is organized into one module per primitive. The core structures are
//! vectors and matrices. A strongly-typed interface is provided, to prevent
//! mixing units or violating mathematical invariants.
//!
//! Transformations are not usually done directly on matrices, but go through
//! transformation objects that can be converted to matrices. Rotations go
//! through the `Basis` types, which are guaranteed to be orthogonal matrices.
//! Despite this, one can directly create a limited rotation matrix using the
//! `look_at`, `from_angle`, `from_euler`, and `from_axis_angle` methods.
//! These are provided for convenience.

#[macro_use]
extern crate approx;

extern crate cgmath;
extern crate num;

#[cfg(feature = "rustc-serialize")]
extern crate rustc_serialize;

// Re-exports
pub use aabb::*;
pub use bound::*;
pub use cylinder::Cylinder;
pub use frustum::{Frustum, FrustumPoints, Projection};
pub use intersect::Intersect;
pub use obb::*;
pub use sphere::Sphere;
pub use plane::Plane;
pub use ray::{Ray, Ray2, Ray3};
pub use line::{Line2, Line3};

// Modules

mod aabb;
mod bound;
mod cylinder;
mod frustum;
mod intersect;
mod obb;
mod sphere;
mod plane;
mod ray;
mod line;
