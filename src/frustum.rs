//! View frustum for visibility determination

use crate::bound::*;
use crate::Plane;
use cgmath::BaseFloat;
use cgmath::Point3;
use cgmath::{Matrix, Matrix4};
use cgmath::{Ortho, Perspective, PerspectiveFov};

/// View frustum, used for frustum culling
#[derive(Copy, Clone, Debug, PartialEq)]
#[cfg_attr(
    feature = "serde",
    derive(Serialize, Deserialize),
    serde(crate = "serde_crate")
)]
pub struct Frustum<S: BaseFloat> {
    /// Left plane
    pub left: Plane<S>,
    /// Right plane
    pub right: Plane<S>,
    /// Bottom plane
    pub bottom: Plane<S>,
    /// Top plane
    pub top: Plane<S>,
    /// Near plane
    pub near: Plane<S>,
    /// Far plane
    pub far: Plane<S>,
}

impl<S: BaseFloat> Frustum<S> {
    /// Construct a frustum.
    pub fn new(
        left: Plane<S>,
        right: Plane<S>,
        bottom: Plane<S>,
        top: Plane<S>,
        near: Plane<S>,
        far: Plane<S>,
    ) -> Frustum<S> {
        Frustum {
            left,
            right,
            bottom,
            top,
            near,
            far,
        }
    }

    /// Extract frustum planes from a projection matrix.
    pub fn from_matrix4(mat: Matrix4<S>) -> Option<Frustum<S>> {
        Some(Frustum::new(
            match Plane::from_vector4_alt(mat.row(3) + mat.row(0)).normalize() {
                Some(p) => p,
                None => return None,
            },
            match Plane::from_vector4_alt(mat.row(3) - mat.row(0)).normalize() {
                Some(p) => p,
                None => return None,
            },
            match Plane::from_vector4_alt(mat.row(3) + mat.row(1)).normalize() {
                Some(p) => p,
                None => return None,
            },
            match Plane::from_vector4_alt(mat.row(3) - mat.row(1)).normalize() {
                Some(p) => p,
                None => return None,
            },
            match Plane::from_vector4_alt(mat.row(3) + mat.row(2)).normalize() {
                Some(p) => p,
                None => return None,
            },
            match Plane::from_vector4_alt(mat.row(3) - mat.row(2)).normalize() {
                Some(p) => p,
                None => return None,
            },
        ))
    }

    /// Find the spatial relation of a bound inside this frustum.
    pub fn contains<B: PlaneBound<S>>(&self, bound: &B) -> Relation {
        [
            self.left,
            self.right,
            self.top,
            self.bottom,
            self.near,
            self.far,
        ]
        .iter()
        .fold(Relation::In, |cur, p| {
            use std::cmp::max;
            let r = bound.relate_plane(*p);
            // If any of the planes are `Out`, the bound is outside.
            // Otherwise, if any are `Cross`, the bound is crossing.
            // Otherwise, the bound is fully inside.
            max(cur, r)
        })
    }
}

/// View frustum corner points
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(
    feature = "serde",
    derive(Serialize, Deserialize),
    serde(crate = "serde_crate")
)]
pub struct FrustumPoints<S> {
    /// Near top left point
    pub near_top_left: Point3<S>,
    /// Near top right point
    pub near_top_right: Point3<S>,
    /// Near bottom left point
    pub near_bottom_left: Point3<S>,
    /// Near bottom right point
    pub near_bottom_right: Point3<S>,
    /// Far top left point
    pub far_top_left: Point3<S>,
    /// Far top right point
    pub far_top_right: Point3<S>,
    /// Far bottom left point
    pub far_bottom_left: Point3<S>,
    /// Far bottom right point
    pub far_bottom_right: Point3<S>,
}

/// Conversion trait for converting cgmath projection types into a view frustum
pub trait Projection<S: BaseFloat>: Into<Matrix4<S>> {
    /// Create a view frustum
    fn to_frustum(&self) -> Frustum<S>;
}

impl<S: BaseFloat> Projection<S> for PerspectiveFov<S> {
    fn to_frustum(&self) -> Frustum<S> {
        // TODO: Could this be faster?
        Frustum::from_matrix4((*self).into()).unwrap()
    }
}

impl<S: BaseFloat> Projection<S> for Perspective<S> {
    fn to_frustum(&self) -> Frustum<S> {
        // TODO: Could this be faster?
        Frustum::from_matrix4((*self).into()).unwrap()
    }
}

impl<S: BaseFloat> Projection<S> for Ortho<S> {
    fn to_frustum(&self) -> Frustum<S> {
        Frustum {
            left: Plane::from_abcd(S::one(), S::zero(), S::zero(), self.left),
            right: Plane::from_abcd(-S::one(), S::zero(), S::zero(), self.right),
            bottom: Plane::from_abcd(S::zero(), S::one(), S::zero(), self.bottom),
            top: Plane::from_abcd(S::zero(), -S::one(), S::zero(), self.top),
            near: Plane::from_abcd(S::zero(), S::zero(), -S::one(), self.near),
            far: Plane::from_abcd(S::zero(), S::zero(), S::one(), self.far),
        }
    }
}
