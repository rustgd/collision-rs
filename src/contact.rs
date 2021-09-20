//! Collision contact manifold

use cgmath::prelude::*;

/// Collision strategy to use for collisions.
///
/// This is used both to specify what collision strategy to use for each shape, and also each
/// found contact will have this returned on it, detailing what data is relevant in the
/// [`Contact`](struct.Contact.html).
#[derive(Debug, PartialEq, Copy, Clone, PartialOrd)]
#[cfg_attr(
    feature = "serde",
    derive(Serialize, Deserialize),
    serde(crate = "serde_crate")
)]
pub enum CollisionStrategy {
    /// Compute full contact manifold for the collision
    FullResolution,

    /// Only report that a collision occurred, skip computing contact information for the collision.
    CollisionOnly,
}

/// Contact manifold for a single collision contact point.
///
/// # Type parameters
///
/// - `P`: cgmath point type
#[derive(Debug, Clone)]
pub struct Contact<P: EuclideanSpace> {
    /// The collision strategy used for this contact.
    pub strategy: CollisionStrategy,

    /// The collision normal. Only applicable if the collision strategy is not `CollisionOnly`
    pub normal: P::Diff,

    /// The penetration depth. Only applicable if the collision strategy is not `CollisionOnly`
    pub penetration_depth: P::Scalar,

    /// The contact point. Only applicable if the collision strategy is not `CollisionOnly`
    pub contact_point: P,

    /// The time of impact, only applicable for continuous collision detection, value is in
    /// range 0.0..1.0
    pub time_of_impact: P::Scalar,
}

impl<P> Contact<P>
where
    P: EuclideanSpace,
    P::Diff: VectorSpace + Zero,
{
    /// Create a new contact manifold, with default collision normal and penetration depth
    pub fn new(strategy: CollisionStrategy) -> Self {
        Self::new_impl(strategy, P::Diff::zero(), P::Scalar::zero())
    }

    /// Create a new contact manifold, with the given collision normal and penetration depth
    pub fn new_impl(
        strategy: CollisionStrategy,
        normal: P::Diff,
        penetration_depth: P::Scalar,
    ) -> Self {
        Self::new_with_point(strategy, normal, penetration_depth, P::origin())
    }

    /// Create a new contact manifold, complete with contact point
    pub fn new_with_point(
        strategy: CollisionStrategy,
        normal: P::Diff,
        penetration_depth: P::Scalar,
        contact_point: P,
    ) -> Self {
        Self {
            strategy,
            normal,
            penetration_depth,
            contact_point,
            time_of_impact: P::Scalar::zero(),
        }
    }
}
