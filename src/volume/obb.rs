//! Oriented bounding boxes

use std::marker::PhantomData;

use cgmath::{Point2, Point3};
use cgmath::{Vector2, Vector3};

/// Generic object bounding box, centered on `center`, aligned with `axis`,
/// and with size `extents`.
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(
    feature = "serde",
    derive(Serialize, Deserialize),
    serde(crate = "serde_crate")
)]
pub struct Obb<S, V, P> {
    /// OBB center point in world space
    pub center: P,
    /// Axis OBB is aligned with
    pub axis: V,
    /// Size of the OBB
    pub extents: V,
    marker: PhantomData<S>,
}

impl<S, V, P> Obb<S, V, P> {
    /// Create a new generic OBB with the given `center`, `axis` and `extents`
    pub fn new(center: P, axis: V, extents: V) -> Self {
        Self {
            center,
            axis,
            extents,
            marker: PhantomData,
        }
    }
}

/// 2D object bounding box
pub type Obb2<S> = Obb<S, Vector2<S>, Point2<S>>;

/// 3D object bounding box
pub type Obb3<S> = Obb<S, Vector3<S>, Point3<S>>;
