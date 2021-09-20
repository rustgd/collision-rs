//! Line segments

use std::fmt;
use std::marker::PhantomData;

use cgmath::prelude::*;
use cgmath::{BaseFloat, BaseNum};
use cgmath::{Point2, Point3};
use cgmath::{Vector2, Vector3};

use crate::prelude::*;
use crate::Ray2;

/// A generic directed line segment from `origin` to `dest`.
#[derive(Copy, Clone, PartialEq)]
#[cfg_attr(
    feature = "serde",
    derive(Serialize, Deserialize),
    serde(crate = "serde_crate")
)]
pub struct Line<S, V, P> {
    /// Origin of the line
    pub origin: P,
    /// Endpoint of the line
    pub dest: P,
    phantom_s: PhantomData<S>,
    phantom_v: PhantomData<V>,
}

impl<S: BaseNum, V: VectorSpace<Scalar = S>, P: EuclideanSpace<Scalar = S, Diff = V>>
    Line<S, V, P>
{
    /// Create a new directed line segment from `origin` to `dest`.
    pub fn new(origin: P, dest: P) -> Line<S, V, P> {
        Line {
            origin,
            dest,
            phantom_v: PhantomData,
            phantom_s: PhantomData,
        }
    }
}

impl<S, V, P: fmt::Debug> fmt::Debug for Line<S, V, P> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_tuple("Line")
            .field(&self.origin)
            .field(&self.dest)
            .finish()
    }
}

/// 2D directed line segment
pub type Line2<S> = Line<S, Vector2<S>, Point2<S>>;

/// 3D directed line segment
pub type Line3<S> = Line<S, Vector3<S>, Point3<S>>;

impl<S: BaseFloat> Discrete<Ray2<S>> for Line2<S> {
    fn intersects(&self, ray: &Ray2<S>) -> bool {
        ray.intersection(self).is_some()
    }
}

impl<S: BaseFloat> Continuous<Ray2<S>> for Line2<S> {
    type Result = Point2<S>;

    fn intersection(&self, ray: &Ray2<S>) -> Option<Self::Result> {
        ray.intersection(self)
    }
}

/// Determines if an intersection between a ray and a line segment is found.
impl<S: BaseFloat> Continuous<Line2<S>> for Ray2<S> {
    type Result = Point2<S>;
    fn intersection(&self, line: &Line2<S>) -> Option<Point2<S>> {
        let ray = self;

        let p = ray.origin;
        let q = line.origin;
        let r = ray.direction;
        let s = Vector2::new(line.dest.x - line.origin.x, line.dest.y - line.origin.y);

        let cross_1 = r.perp_dot(s);
        let qmp = Vector2::new(q.x - p.x, q.y - p.y);
        let cross_2 = qmp.perp_dot(r);

        if cross_1 == S::zero() {
            if cross_2 != S::zero() {
                // parallel
                return None;
            }

            // collinear
            let q2mp = Vector2::new(line.dest.x - p.x, line.dest.y - p.y);
            let dot_1 = qmp.dot(r);
            let dot_2 = q2mp.dot(r);
            if (dot_1 <= S::zero() && dot_2 >= S::zero())
                || (dot_1 >= S::zero() && dot_2 <= S::zero())
            {
                return Some(p);
            } else if dot_1 >= S::zero() && dot_2 >= S::zero() {
                if dot_1 <= dot_2 {
                    return Some(q);
                } else {
                    return Some(line.dest);
                }
            }

            // no overlap exists
            return None;
        }

        let t = qmp.perp_dot(s) / cross_1;
        let u = cross_2 / cross_1;

        if S::zero() <= t && u >= S::zero() && u <= S::one() {
            return Some(Point2::new(p.x + t * r.x, p.y + t * r.y));
        }

        None
    }
}
