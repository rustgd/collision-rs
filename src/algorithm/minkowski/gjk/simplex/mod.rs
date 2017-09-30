pub use self::simplex2d::SimplexProcessor2;
pub use self::simplex3d::SimplexProcessor3;

mod simplex2d;
mod simplex3d;

use cgmath::prelude::*;

use algorithm::minkowski::SupportPoint;

/// Defined a simplex processor for use in GJK.
pub trait SimplexProcessor {
    /// The point type of the processor
    type Point: EuclideanSpace;

    /// Check if the given simplex contains origin, and if not, update the simplex and search
    /// direction
    fn check_origin(
        &self,
        simplex: &mut Vec<SupportPoint<Self::Point>>,
        d: &mut <Self::Point as EuclideanSpace>::Diff,
    ) -> bool;

    /// Create a new simplex processor
    fn new() -> Self;
}
