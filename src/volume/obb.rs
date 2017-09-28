// Copyright 2013 The CGMath Developers. For a full listing of the authors,
// refer to the Cargo.toml file at the top-level directory of this distribution.
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

//! Oriented bounding boxes

use std::marker::PhantomData;

use cgmath::{Point2, Point3};
use cgmath::{Vector2, Vector3};

/// Generic object bounding box, centered on `center`, aligned with `axis`,
/// and with size `extents`.
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "eders", derive(Serialize, Deserialize))]
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
