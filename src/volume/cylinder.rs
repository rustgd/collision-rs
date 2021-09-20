//! Oriented bounding cylinder

use cgmath::Point3;
use cgmath::Vector3;

/// Bounding cylinder
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(
    feature = "serde",
    derive(Serialize, Deserialize),
    serde(crate = "serde_crate")
)]
pub struct Cylinder<S> {
    /// Center point
    pub center: Point3<S>,
    /// Axis the cylinder is aligned with
    pub axis: Vector3<S>,
    /// Radius of the cylinder
    pub radius: S,
}
