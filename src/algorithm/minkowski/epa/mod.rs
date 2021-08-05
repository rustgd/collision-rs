//! Expanding Polytope Algorithm

pub use self::epa2d::EPA2;
pub use self::epa3d::EPA3;

mod epa2d;
mod epa3d;

use cgmath::prelude::*;

use super::SupportPoint;
use crate::prelude::*;
use crate::Contact;

pub const EPA_TOLERANCE: f32 = 0.00001;
pub const MAX_ITERATIONS: u32 = 100;

/// Expanding Polytope Algorithm base trait
pub trait EPA {
    /// Point type
    type Point: EuclideanSpace;

    /// Process the given simplex, and compute the contact point.
    ///
    /// The given simplex must be a complete simplex for the given space, and it must enclose the
    /// origin.
    fn process<SL, SR, TL, TR>(
        &self,
        simplex: &mut Vec<SupportPoint<Self::Point>>,
        left: &SL,
        left_transform: &TL,
        right: &SR,
        right_transform: &TR,
    ) -> Option<Contact<Self::Point>>
    where
        SL: Primitive<Point = Self::Point>,
        SR: Primitive<Point = Self::Point>,
        TL: Transform<Self::Point>,
        TR: Transform<Self::Point>;

    /// Create a new EPA instance
    fn new() -> Self;

    /// Create a new EPA instance with the given tolerance
    fn new_with_tolerance(
        tolerance: <Self::Point as EuclideanSpace>::Scalar,
        max_iterations: u32,
    ) -> Self;
}
