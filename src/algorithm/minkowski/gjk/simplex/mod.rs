pub use self::simplex2d::SimplexProcessor2;
pub use self::simplex3d::SimplexProcessor3;

mod simplex2d;
mod simplex3d;

use cgmath::prelude::*;
use smallvec::SmallVec;

use crate::algorithm::minkowski::SupportPoint;

pub type Simplex<P> = SmallVec<[SupportPoint<P>; 5]>;

/// Defined a simplex processor for use in GJK.
pub trait SimplexProcessor {
    /// The point type of the processor
    type Point: EuclideanSpace;

    /// Check if the given simplex contains origin, and if not, update the simplex and search
    /// direction.
    ///
    /// Used by the GJK intersection test
    fn reduce_to_closest_feature(
        &self,
        simplex: &mut Simplex<Self::Point>,
        d: &mut <Self::Point as EuclideanSpace>::Diff,
    ) -> bool;

    /// Get the closest point on the simplex to the origin.
    ///
    /// Will also update the simplex to contain only the feature that is closest to the origin.
    /// This will be an edge for 2D; and it will be an edge or a face for 3D.
    /// This is primarily used by the GJK distance computation.
    fn get_closest_point_to_origin(
        &self,
        simplex: &mut Simplex<Self::Point>,
    ) -> <Self::Point as EuclideanSpace>::Diff;

    /// Create a new simplex processor
    fn new() -> Self;
}
