#![crate_type = "rlib"]
#![crate_type = "dylib"]
#![forbid(unsafe_code, unstable_features)]
#![deny(
    missing_docs,
    missing_copy_implementations,
    missing_debug_implementations,
    trivial_casts,
    unused_import_braces,
    unused_qualifications,
    rust_2018_compatibility,
    rust_2018_idioms,
    nonstandard_style,
    unused,
    future_incompatible,
    clippy::semicolon_if_nothing_returned,
    clippy::unreadable_literal,
    clippy::unseparated_literal_suffix,
    clippy::needless_pass_by_value
)]
#![allow(clippy::excessive_precision)]

//! Companion library to cgmath, dealing with collision detection centric data structures and
//! algorithms.
//!
//! This crate provides useful data structures and algorithms for doing collision detection.
//! It is organized into a few distinct parts: generic geometry (ray, line, plane, frustum etc),
//! bounding volumes (AABB, OBB, Sphere etc), collision primitives and algorithms used for
//! collision detection, distance computation etc.
//!

#[cfg(feature = "serde")]
#[macro_use]
extern crate serde_crate;

// Re-exports

pub use bound::*;
pub use contact::*;
pub use frustum::*;
pub use line::*;
pub use plane::Plane;
pub use ray::*;
pub use traits::*;
pub use volume::*;

pub mod algorithm;
pub mod dbvt;
pub mod prelude;
pub mod primitive;

// Modules

mod bound;
mod contact;
mod frustum;
mod line;
mod plane;
mod ray;
mod traits;
mod volume;
