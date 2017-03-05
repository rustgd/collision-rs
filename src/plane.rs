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

use std::fmt;

use cgmath::{ApproxEq, BaseFloat};
use cgmath::{Point3};
use cgmath::{Vector3, Vector4};
use cgmath::{EuclideanSpace, InnerSpace};
use cgmath::{Zero};


/// A 3-dimensional plane formed from the equation: `A*x + B*y + C*z - D = 0`.
///
/// # Fields
///
/// - `n`: a unit vector representing the normal of the plane where:
///   - `n.x`: corresponds to `A` in the plane equation
///   - `n.y`: corresponds to `B` in the plane equation
///   - `n.z`: corresponds to `C` in the plane equation
/// - `d`: the distance value, corresponding to `D` in the plane equation
///
/// # Notes
///
/// The `A*x + B*y + C*z - D = 0` form is preferred over the other common
/// alternative, `A*x + B*y + C*z + D = 0`, because it tends to avoid
/// superfluous negations (see _Real Time Collision Detection_, p. 55).
#[derive(Copy, Clone, PartialEq)]
#[cfg_attr(feature = "rustc-serialize", derive(RustcEncodable, RustcDecodable))]
pub struct Plane<S> {
    pub n: Vector3<S>,
    pub d: S,
}

impl<S: BaseFloat> Plane<S> {
    /// Construct a plane from a normal vector and a scalar distance. The
    /// plane will be perpendicular to `n`, and `d` units offset from the
    /// origin.
    pub fn new(n: Vector3<S>, d: S) -> Plane<S> {
        Plane { n: n, d: d }
    }

    /// # Arguments
    ///
    /// - `a`: the `x` component of the normal
    /// - `b`: the `y` component of the normal
    /// - `c`: the `z` component of the normal
    /// - `d`: the plane's distance value
    pub fn from_abcd(a: S, b: S, c: S, d: S) -> Plane<S> {
        Plane { n: Vector3::new(a, b, c), d: d }
    }

    /// Construct a plane from the components of a four-dimensional vector
    pub fn from_vector4(v: Vector4<S>) -> Plane<S> {
        Plane { n: Vector3::new(v.x, v.y, v.z), d: v.w }
    }

    /// Construct a plane from the components of a four-dimensional vector
    /// Assuming alternative representation: `A*x + B*y + C*z + D = 0`
    pub fn from_vector4_alt(v: Vector4<S>) -> Plane<S> {
        Plane { n: Vector3::new(v.x, v.y, v.z), d: -v.w }
    }

    /// Constructs a plane that passes through the the three points `a`, `b` and `c`
    pub fn from_points(a: Point3<S>, b: Point3<S>, c: Point3<S>) -> Option<Plane<S>> {
        // create two vectors that run parallel to the plane
        let v0 = b - a;
        let v1 = c - a;

        // find the normal vector that is perpendicular to v1 and v2
        let n = v0.cross(v1);

        if ulps_eq!(n, &Vector3::zero()) { None }
        else {
            // compute the normal and the distance to the plane
            let n = n.normalize();
            let d = -a.dot(n);

            Some(Plane::new(n, d))
        }
    }

    /// Construct a plane from a point and a normal vector.
    /// The plane will contain the point `p` and be perpendicular to `n`.
    pub fn from_point_normal(p: Point3<S>, n: Vector3<S>) -> Plane<S> {
        Plane { n: n, d: p.dot(n) }
    }

    /// Normalize a plane.
    pub fn normalize(&self) -> Option<Plane<S>> {
        if ulps_eq!(self.n, &Vector3::zero()) { None }
        else {
            let denom = S::one() / self.n.magnitude();
            Some(Plane::new(self.n * denom, self.d*denom))
        }
    }
}

impl<S> ApproxEq for Plane<S>
    // where S: BaseFloat + ApproxEq<Epsilon=S>
    where S: BaseFloat
{
    type Epsilon = S::Epsilon;

    #[inline]
    fn default_epsilon() -> S::Epsilon {
        S::default_epsilon()
    }

    #[inline]
    fn default_max_relative() -> S::Epsilon {
        S::default_max_relative()
    }

    #[inline]
    fn default_max_ulps() -> u32 {
        S::default_max_ulps()
    }

    #[inline]
    fn relative_eq(&self, other: &Self, epsilon: S::Epsilon, max_relative: S::Epsilon) -> bool {
        Vector3::relative_eq(&self.n, &other.n, epsilon, max_relative) &&
        S::relative_eq(&self.d, &other.d, epsilon, max_relative)
    }

    #[inline]
    fn ulps_eq(&self, other: &Self, epsilon: S::Epsilon, max_ulps: u32) -> bool {
        Vector3::ulps_eq(&self.n, &other.n, epsilon, max_ulps) &&
        S::ulps_eq(&self.d, &other.d, epsilon, max_ulps)
    }
}

impl<S: BaseFloat> fmt::Debug for Plane<S> {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "{:?}x + {:?}y + {:?}z - {:?} = 0",
               self.n.x, self.n.y, self.n.z, self.d)
    }
}
