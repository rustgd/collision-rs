//! Generic spatial bounds.

use cgmath::BaseFloat;
use cgmath::Matrix4;
use cgmath::{EuclideanSpace, Point3};
use std::{cmp, fmt};

use crate::frustum::Frustum;
use crate::plane::Plane;

/// Spatial relation between two objects.
#[derive(Copy, Clone, Debug, Eq, Hash, Ord, PartialOrd, PartialEq)]
#[repr(u8)]
pub enum Relation {
    /// Completely inside.
    In,
    /// Crosses the boundary.
    Cross,
    /// Completely outside.
    Out,
}

/// Generic 3D bound.
pub trait PlaneBound<S: BaseFloat>: fmt::Debug {
    /// Classify the spatial relation with a plane.
    fn relate_plane(&self, plane: Plane<S>) -> Relation;
    /// Classify the relation with a projection matrix.
    fn relate_clip_space(&self, projection: Matrix4<S>) -> Relation {
        let frustum = match Frustum::from_matrix4(projection) {
            Some(f) => f,
            None => return Relation::Cross,
        };
        [
            frustum.left,
            frustum.right,
            frustum.top,
            frustum.bottom,
            frustum.near,
            frustum.far,
        ]
        .iter()
        .fold(Relation::In, |cur, p| {
            let r = self.relate_plane(*p);
            // If any of the planes are `Out`, the bound is outside.
            // Otherwise, if any are `Cross`, the bound is crossing.
            // Otherwise, the bound is fully inside.
            cmp::max(cur, r)
        })
    }
}

impl<S: BaseFloat> PlaneBound<S> for Point3<S> {
    fn relate_plane(&self, plane: Plane<S>) -> Relation {
        let dist = self.dot(plane.n);
        if dist > plane.d {
            Relation::In
        } else if dist < plane.d {
            Relation::Out
        } else {
            Relation::Cross
        }
    }

    fn relate_clip_space(&self, projection: Matrix4<S>) -> Relation {
        use std::cmp::Ordering::*;
        let p = projection * self.to_homogeneous();
        match (
            p.x.abs().partial_cmp(&p.w),
            p.y.abs().partial_cmp(&p.w),
            p.z.abs().partial_cmp(&p.w),
        ) {
            (Some(Less), Some(Less), Some(Less)) => Relation::In,
            (Some(Greater), _, _) | (_, Some(Greater), _) | (_, _, Some(Greater)) => Relation::Out,
            _ => Relation::Cross,
        }
    }
}
