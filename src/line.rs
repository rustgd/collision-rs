// Copyright 2013-2014 The CGMath Developers. For a full listing of the authors,
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

//! Line segments

use std::marker::PhantomData;

use cgmath::{BaseNum};
use cgmath::{EuclideanSpace, Point2, Point3};
use cgmath::{VectorSpace, Vector2, Vector3};

/// A generic directed line segment from `origin` to `dest`.
#[derive(Copy, Clone, PartialEq)]
#[cfg_attr(feature = "rustc-serialize", derive(RustcEncodable, RustcDecodable))]
pub struct Line<S, V, P> {
    pub origin: P,
    pub dest: P,
    phantom_s: PhantomData<S>,
    phantom_v: PhantomData<V>
}

impl<S: BaseNum, V: VectorSpace<Scalar=S>, P: EuclideanSpace<Scalar=S, Diff=V>>  Line<S, V, P> {
    pub fn new(origin: P, dest: P) -> Line<S, V, P> {
        Line {
            origin: origin,
            dest: dest,
            phantom_v: PhantomData,
            phantom_s: PhantomData
        }
    }
}

pub type Line2<S> = Line<S, Vector2<S>, Point2<S>>;
pub type Line3<S> = Line<S, Vector3<S>, Point3<S>>;
