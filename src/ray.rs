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

use std::marker::PhantomData;
use cgmath::BaseNum;
use cgmath::{EuclideanSpace, Point2, Point3};
use cgmath::{VectorSpace, Vector2, Vector3};

/// A generic ray starting at `origin` and extending infinitely in
/// `direction`.
#[derive(Copy, Clone, PartialEq)]
#[cfg_attr(feature = "eders", derive(Serialize, Deserialize))]
pub struct Ray<S, P, V> {
    pub origin: P,
    pub direction: V,
    phantom_s: PhantomData<S>,
}

impl<S, V, P> Ray<S, P, V>
where
    S: BaseNum,
    V: VectorSpace<Scalar = S>,
    P: EuclideanSpace<Scalar = S, Diff = V>,
{
    pub fn new(origin: P, direction: V) -> Ray<S, P, V> {
        Ray {
            origin: origin,
            direction: direction,
            phantom_s: PhantomData,
        }
    }
}

pub type Ray2<S> = Ray<S, Point2<S>, Vector2<S>>;
pub type Ray3<S> = Ray<S, Point3<S>, Vector3<S>>;
