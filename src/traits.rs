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

use cgmath::BaseNum;

pub trait Continuous<RHS> {
    type Result;

    fn intersection(&self, &RHS) -> Option<Self::Result>;
}

pub trait Discrete<RHS> {
    fn intersects(&self, &RHS) -> bool;
}

pub trait Contains<RHS> {
    #[inline]
    fn contains(&self, &RHS) -> bool;
}

pub trait SurfaceArea<S: BaseNum> {
    fn surface_area(&self) -> S;
}

pub trait Union<RHS = Self> {
    type Output;

    fn union(&self, &RHS) -> Self::Output;
}
