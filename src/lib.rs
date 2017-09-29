#![crate_type = "rlib"]
#![crate_type = "dylib"]
#![deny(missing_docs, missing_debug_implementations, trivial_casts, unsafe_code, unstable_features,
       unused_import_braces, unused_qualifications)]

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

extern crate bit_set;
extern crate cgmath;
extern crate num;
extern crate rand;

#[cfg(feature = "eders")]
extern crate serde;
#[cfg(feature = "eders")]
#[macro_use]
extern crate serde_derive;

// Re-exports

pub use bound::*;
pub use frustum::*;
pub use line::*;
pub use plane::Plane;
pub use ray::*;
pub use traits::*;
pub use volume::*;

pub mod prelude;
pub mod dbvt;
pub mod primitive;

// Modules

mod bound;
mod frustum;
mod traits;
mod plane;
mod ray;
mod line;
mod volume;
