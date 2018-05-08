#![crate_type = "rlib"]
#![crate_type = "dylib"]
#![deny(missing_docs, trivial_casts, unsafe_code, unstable_features, unused_import_braces,
        unused_qualifications)]

//! Companion library to cgmath, dealing with collision detection centric data structures and
//! algorithms.
//!
//! This crate provides useful data structures and algorithms for doing collision detection.
//! It is organized into a few distinct parts: generic geometry (ray, line, plane, frustum etc),
//! bounding volumes (AABB, OBB, Sphere etc), collision primitives and algorithms used for
//! collision detection, distance computation etc.
//!
#[macro_use]
extern crate approx;

extern crate bit_set;
extern crate cgmath;
extern crate num;
extern crate rand;

#[cfg(feature = "serde")]
#[macro_use]
extern crate serde;

// Re-exports

pub use bound::*;
pub use contact::*;
pub use frustum::*;
pub use line::*;
pub use plane::Plane;
pub use ray::*;
pub use traits::*;
pub use volume::*;

pub mod prelude;
pub mod dbvt;
pub mod primitive;
pub mod algorithm;

// Modules

mod bound;
mod frustum;
mod traits;
mod plane;
mod ray;
mod line;
mod volume;
mod contact;
