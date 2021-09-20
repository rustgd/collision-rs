use std::fmt;

use cgmath::prelude::*;
use cgmath::{ulps_eq, ulps_ne};
use cgmath::{AbsDiffEq, RelativeEq, UlpsEq};
use cgmath::{BaseFloat, Point3, Vector3, Vector4};

use crate::prelude::*;
use crate::Ray3;

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
#[cfg_attr(
    feature = "serde",
    derive(Serialize, Deserialize),
    serde(crate = "serde_crate")
)]
pub struct Plane<S> {
    /// Plane normal
    pub n: Vector3<S>,
    /// Plane distance value
    pub d: S,
}

impl<S: BaseFloat> Plane<S> {
    /// Construct a plane from a normal vector and a scalar distance. The
    /// plane will be perpendicular to `n`, and `d` units offset from the
    /// origin.
    pub fn new(n: Vector3<S>, d: S) -> Plane<S> {
        Plane { n, d }
    }

    /// # Arguments
    ///
    /// - `a`: the `x` component of the normal
    /// - `b`: the `y` component of the normal
    /// - `c`: the `z` component of the normal
    /// - `d`: the plane's distance value
    pub fn from_abcd(a: S, b: S, c: S, d: S) -> Plane<S> {
        Plane {
            n: Vector3::new(a, b, c),
            d,
        }
    }

    /// Construct a plane from the components of a four-dimensional vector
    pub fn from_vector4(v: Vector4<S>) -> Plane<S> {
        Plane {
            n: Vector3::new(v.x, v.y, v.z),
            d: v.w,
        }
    }

    /// Construct a plane from the components of a four-dimensional vector
    /// Assuming alternative representation: `A*x + B*y + C*z + D = 0`
    pub fn from_vector4_alt(v: Vector4<S>) -> Plane<S> {
        Plane {
            n: Vector3::new(v.x, v.y, v.z),
            d: -v.w,
        }
    }

    /// Constructs a plane that passes through the the three points `a`, `b` and `c`
    pub fn from_points(a: Point3<S>, b: Point3<S>, c: Point3<S>) -> Option<Plane<S>> {
        // create two vectors that run parallel to the plane
        let v0 = b - a;
        let v1 = c - a;

        // find the normal vector that is perpendicular to v1 and v2
        let n = v0.cross(v1);

        if ulps_eq!(n, &Vector3::zero()) {
            None
        } else {
            // compute the normal and the distance to the plane
            let n = n.normalize();
            let d = -a.dot(n);

            Some(Plane::new(n, d))
        }
    }

    /// Construct a plane from a point and a normal vector.
    /// The plane will contain the point `p` and be perpendicular to `n`.
    pub fn from_point_normal(p: Point3<S>, n: Vector3<S>) -> Plane<S> {
        Plane { n, d: p.dot(n) }
    }

    /// Normalize a plane.
    pub fn normalize(&self) -> Option<Plane<S>> {
        if ulps_eq!(self.n, &Vector3::zero()) {
            None
        } else {
            let denom = S::one() / self.n.magnitude();
            Some(Plane::new(self.n * denom, self.d * denom))
        }
    }
}

impl<S: AbsDiffEq> AbsDiffEq for Plane<S>
where
    S::Epsilon: Copy,
    S: BaseFloat,
{
    type Epsilon = S::Epsilon;

    #[inline]
    fn default_epsilon() -> S::Epsilon {
        S::default_epsilon()
    }

    #[inline]
    fn abs_diff_eq(&self, other: &Self, epsilon: S::Epsilon) -> bool {
        Vector3::abs_diff_eq(&self.n, &other.n, epsilon)
            && S::abs_diff_eq(&self.d, &other.d, epsilon)
    }
}

impl<S: RelativeEq> RelativeEq for Plane<S>
where
    S::Epsilon: Copy,
    S: BaseFloat,
{
    #[inline]
    fn default_max_relative() -> S::Epsilon {
        S::default_max_relative()
    }

    #[inline]
    fn relative_eq(&self, other: &Self, epsilon: S::Epsilon, max_relative: S::Epsilon) -> bool {
        Vector3::relative_eq(&self.n, &other.n, epsilon, max_relative)
            && S::relative_eq(&self.d, &other.d, epsilon, max_relative)
    }
}
impl<S: UlpsEq> UlpsEq for Plane<S>
where
    S::Epsilon: Copy,
    S: BaseFloat,
{
    #[inline]
    fn default_max_ulps() -> u32 {
        S::default_max_ulps()
    }

    #[inline]
    fn ulps_eq(&self, other: &Self, epsilon: S::Epsilon, max_ulps: u32) -> bool {
        Vector3::ulps_eq(&self.n, &other.n, epsilon, max_ulps)
            && S::ulps_eq(&self.d, &other.d, epsilon, max_ulps)
    }
}

impl<S: BaseFloat> fmt::Debug for Plane<S> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "{:?}x + {:?}y + {:?}z - {:?} = 0",
            self.n.x, self.n.y, self.n.z, self.d
        )
    }
}

impl<S: BaseFloat> Continuous<Ray3<S>> for Plane<S> {
    type Result = Point3<S>;
    fn intersection(&self, r: &Ray3<S>) -> Option<Point3<S>> {
        let p = self;

        let t = -(p.d + r.origin.dot(p.n)) / r.direction.dot(p.n);
        if t < Zero::zero() {
            None
        } else {
            Some(r.origin + r.direction * t)
        }
    }
}

impl<S: BaseFloat> Discrete<Ray3<S>> for Plane<S> {
    fn intersects(&self, r: &Ray3<S>) -> bool {
        let p = self;
        let t = -(p.d + r.origin.dot(p.n)) / r.direction.dot(p.n);
        t >= Zero::zero()
    }
}

/// See _Real-Time Collision Detection_, p. 210
impl<S: BaseFloat> Continuous<Plane<S>> for Plane<S> {
    type Result = Ray3<S>;
    fn intersection(&self, p2: &Plane<S>) -> Option<Ray3<S>> {
        let p1 = self;
        let d = p1.n.cross(p2.n);
        let denom = d.dot(d);
        if ulps_eq!(denom, &S::zero()) {
            None
        } else {
            let p = (p2.n * p1.d - p1.n * p2.d).cross(d) / denom;
            Some(Ray3::new(Point3::from_vec(p), d))
        }
    }
}

impl<S: BaseFloat> Discrete<Plane<S>> for Plane<S> {
    fn intersects(&self, p2: &Plane<S>) -> bool {
        let p1 = self;
        let d = p1.n.cross(p2.n);
        let denom = d.dot(d);
        ulps_ne!(denom, &S::zero())
    }
}

/// See _Real-Time Collision Detection_, p. 212 - 214
impl<S: BaseFloat> Continuous<(Plane<S>, Plane<S>)> for Plane<S> {
    type Result = Point3<S>;
    fn intersection(&self, planes: &(Plane<S>, Plane<S>)) -> Option<Point3<S>> {
        let (p1, p2, p3) = (self, planes.0, planes.1);
        let u = p2.n.cross(p3.n);
        let denom = p1.n.dot(u);
        if ulps_eq!(denom.abs(), &S::zero()) {
            None
        } else {
            let p = (u * p1.d + p1.n.cross(p2.n * p3.d - p3.n * p2.d)) / denom;
            Some(Point3::from_vec(p))
        }
    }
}

impl<S: BaseFloat> Discrete<(Plane<S>, Plane<S>)> for Plane<S> {
    fn intersects(&self, planes: &(Plane<S>, Plane<S>)) -> bool {
        let (p1, p2, p3) = (self, planes.0, planes.1);
        let u = p2.n.cross(p3.n);
        let denom = p1.n.dot(u);
        ulps_ne!(denom.abs(), &S::zero())
    }
}
