use cgmath::prelude::*;
use cgmath::BaseNum;

/// An intersection test with a result.
///
/// An example would be a Ray vs AABB intersection test that returns a Point in space.
///
pub trait Continuous<RHS> {
    /// Result returned by the intersection test
    type Result;

    /// Intersection test
    fn intersection(&self, _: &RHS) -> Option<Self::Result>;
}

/// A boolean intersection test.
///
pub trait Discrete<RHS> {
    /// Intersection test
    fn intersects(&self, _: &RHS) -> bool;
}

/// Boolean containment test.
///
pub trait Contains<RHS> {
    /// Containment test
    fn contains(&self, _: &RHS) -> bool;
}

/// Shape surface area
///
pub trait SurfaceArea {
    /// Result type returned from surface area computation
    type Scalar: BaseNum;

    /// Compute surface area
    fn surface_area(&self) -> Self::Scalar;
}

/// Build the union of two shapes.
///
pub trait Union<RHS = Self> {
    /// Union shape created
    type Output;

    /// Build the union shape of self and the given shape.
    fn union(&self, _: &RHS) -> Self::Output;
}

/// Bounding volume abstraction for use with algorithms
pub trait Bound {
    /// Point type for the bounding volume (for dimensionality)
    type Point: EuclideanSpace;

    /// Minimum extents of the bounding volume
    fn min_extent(&self) -> Self::Point;
    /// Maximum extents of the bounding volume
    fn max_extent(&self) -> Self::Point;
    /// Create a new bounding volume extended by the given amount
    fn with_margin(&self, add: <Self::Point as EuclideanSpace>::Diff) -> Self;
    /// Apply an arbitrary transform to the bounding volume
    fn transform_volume<T>(&self, transform: &T) -> Self
    where
        T: Transform<Self::Point>;
    /// Create empty volume
    fn empty() -> Self;
}

/// Primitive with bounding volume
pub trait HasBound {
    /// Bounding volume type
    type Bound: Bound;

    /// Borrow the bounding volume
    fn bound(&self) -> &Self::Bound;
}

/// Utilities for computing bounding volumes of primitives
pub trait ComputeBound<B>
where
    B: Bound,
{
    /// Compute the bounding volume
    fn compute_bound(&self) -> B;
}

/// Minkowski support function for primitive
pub trait Primitive {
    /// Point type
    type Point: EuclideanSpace;

    /// Get the support point on the shape in a given direction.
    ///
    /// ## Parameters
    ///
    /// - `direction`: The search direction in world space.
    /// - `transform`: The current local to world transform for this primitive.
    ///
    /// ## Returns
    ///
    /// Return the point that is furthest away from the origin, in the given search direction.
    /// For discrete shapes, the furthest vertex is enough, there is no need to do exact
    /// intersection point computation.
    ///
    /// ## Type parameters
    ///
    /// - `P`: Transform type
    fn support_point<T>(
        &self,
        direction: &<Self::Point as EuclideanSpace>::Diff,
        transform: &T,
    ) -> Self::Point
    where
        T: Transform<Self::Point>;
}

/// Discrete intersection test on transformed primitive
pub trait DiscreteTransformed<RHS> {
    /// Point type for transformation of self
    type Point: EuclideanSpace;

    /// Intersection test for transformed self
    fn intersects_transformed<T>(&self, _: &RHS, _: &T) -> bool
    where
        T: Transform<Self::Point>;
}

/// Continuous intersection test on transformed primitive
pub trait ContinuousTransformed<RHS> {
    /// Point type for transformation of self
    type Point: EuclideanSpace;

    /// Result of intersection test
    type Result: EuclideanSpace;

    /// Intersection test for transformed self
    fn intersection_transformed<T>(&self, _: &RHS, _: &T) -> Option<Self::Result>
    where
        T: Transform<Self::Point>;
}

/// Trait used for interpolation of values
///
/// ## Type parameters:
///
/// - `S`: The scalar type used for amount
pub trait Interpolate<S> {
    /// Interpolate between `self` and `other`, using amount to calculate how much of other to use.
    ///
    /// ## Parameters:
    ///
    /// - `amount`: amount in the range 0. .. 1.
    /// - `other`: the other value to interpolate with
    ///
    /// ## Returns
    ///
    /// A new value approximately equal to `self * (1. - amount) + other * amount`.
    fn interpolate(&self, other: &Self, amount: S) -> Self;
}

/// Trait used for interpolation of translation only in transforms
pub trait TranslationInterpolate<S> {
    /// Interpolate between `self` and `other`, using amount to calculate how much of other to use.
    ///
    /// ## Parameters:
    ///
    /// - `amount`: amount in the range 0. .. 1.
    /// - `other`: the other value to interpolate with
    ///
    /// ## Returns
    ///
    /// A new value approximately equal to `self * (1. - amount) + other * amount`.
    fn translation_interpolate(&self, other: &Self, amount: S) -> Self;
}

mod interpolate {
    use super::{Interpolate, TranslationInterpolate};
    use cgmath::prelude::*;
    use cgmath::{BaseFloat, Basis2, Basis3, Decomposed, Quaternion, Rad};

    impl<S> Interpolate<S> for Quaternion<S>
    where
        S: BaseFloat,
    {
        fn interpolate(&self, other: &Self, amount: S) -> Self {
            self.lerp(*other, amount)
        }
    }

    impl<S> Interpolate<S> for Basis3<S>
    where
        S: BaseFloat,
    {
        fn interpolate(&self, other: &Self, amount: S) -> Self {
            Basis3::from(
                Quaternion::from(*self.as_ref()).lerp(Quaternion::from(*other.as_ref()), amount),
            )
        }
    }

    impl<S> Interpolate<S> for Basis2<S>
    where
        S: BaseFloat,
    {
        fn interpolate(&self, other: &Self, amount: S) -> Self {
            // to complex numbers
            let self_mat = self.as_ref();
            let other_mat = other.as_ref();
            let self_c = self_mat.x;
            let other_c = other_mat.x;
            // do interpolation
            let c = self_c.lerp(other_c, amount);
            // to basis
            Rotation2::from_angle(Rad(c.x.acos()))
        }
    }

    impl<V, R> Interpolate<V::Scalar> for Decomposed<V, R>
    where
        V: VectorSpace + InnerSpace,
        R: Interpolate<V::Scalar>,
        V::Scalar: BaseFloat,
    {
        fn interpolate(&self, other: &Self, amount: V::Scalar) -> Self {
            Decomposed {
                disp: self.disp.lerp(other.disp, amount),
                rot: self.rot.interpolate(&other.rot, amount),
                scale: self.scale * (V::Scalar::one() - amount) + other.scale * amount,
            }
        }
    }

    impl<V, R> TranslationInterpolate<V::Scalar> for Decomposed<V, R>
    where
        V: VectorSpace + InnerSpace,
        R: Clone,
        V::Scalar: BaseFloat,
    {
        fn translation_interpolate(&self, other: &Self, amount: V::Scalar) -> Self {
            Decomposed {
                disp: self.disp.lerp(other.disp, amount),
                rot: other.rot.clone(),
                scale: other.scale,
            }
        }
    }
}
