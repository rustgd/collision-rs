// Copyright 2013-2014 The CGMath Developers. For a full listing of the authors,
// refer to the AUTHORS file at the top-level directory of this distribution.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//! Generic spatial bounds.

use Plane;
use cgmath::Matrix4;
use cgmath::BaseFloat;
use cgmath::{EuclideanSpace, Point3};

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

/// Generic bound.
pub trait Bound<S: BaseFloat + 'static>: Sized + Copy {
    /// Classify the spatial relation with a plane.
    fn relate_plane(self, Plane<S>) -> Relation;
    /// Classify the relation with a projection matrix.
    fn relate_clip_space(self, projection: Matrix4<S>) -> Relation {
        use frustum::Frustum;
        match Frustum::from_matrix4(projection) {
            Some(f) => f.contains(self),
            None => Relation::Cross,
        }
    }
}

impl<S: BaseFloat + 'static> Bound<S> for Point3<S> {
    fn relate_plane(self, plane: Plane<S>) -> Relation {
        let dist = self.dot(plane.n);
        if dist > plane.d {
            Relation::In
        } else if dist < plane.d {
            Relation::Out
        } else {
            Relation::Cross
        }
    }

    fn relate_clip_space(self, projection: Matrix4<S>) -> Relation {
        use std::cmp::Ordering::*;
        let p = projection * self.to_homogeneous();
        match (
            p.x.abs().partial_cmp(&p.w),
            p.y.abs().partial_cmp(&p.w),
            p.z.abs().partial_cmp(&p.w),
        ) {
            (Some(Less), Some(Less), Some(Less)) => Relation::In,
            (Some(Greater), _, _) |
            (_, Some(Greater), _) |
            (_, _, Some(Greater)) => Relation::Out,
            _ => Relation::Cross,
        }
    }
}
