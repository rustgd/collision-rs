//! Axis aligned bounding box for 2D.
//!

use std::fmt;

use cgmath::prelude::*;
use cgmath::{BaseFloat, BaseNum, Point2, Vector2};

use super::{max, min};
use crate::prelude::*;
use crate::{Line2, Ray2};

/// A two-dimensional AABB, aka a rectangle.
#[derive(Copy, Clone, PartialEq)]
#[cfg_attr(
    feature = "serde",
    derive(Serialize, Deserialize),
    serde(crate = "serde_crate")
)]
pub struct Aabb2<S> {
    /// Minimum point of the AABB
    pub min: Point2<S>,
    /// Maximum point of the AABB
    pub max: Point2<S>,
}

impl<S: BaseNum> Aabb2<S> {
    /// Construct a new axis-aligned bounding box from two points.
    #[inline]
    pub fn new(p1: Point2<S>, p2: Point2<S>) -> Aabb2<S> {
        Aabb2 {
            min: Point2::new(min(p1.x, p2.x), min(p1.y, p2.y)),
            max: Point2::new(max(p1.x, p2.x), max(p1.y, p2.y)),
        }
    }

    /// Compute corners.
    #[inline]
    pub fn to_corners(&self) -> [Point2<S>; 4] {
        [
            self.min,
            Point2::new(self.max.x, self.min.y),
            Point2::new(self.min.x, self.max.y),
            self.max,
        ]
    }
}

impl<S: BaseNum> Aabb for Aabb2<S> {
    type Scalar = S;
    type Diff = Vector2<S>;
    type Point = Point2<S>;

    #[inline]
    fn new(p1: Point2<S>, p2: Point2<S>) -> Aabb2<S> {
        Aabb2::new(p1, p2)
    }

    #[inline]
    fn min(&self) -> Point2<S> {
        self.min
    }

    #[inline]
    fn max(&self) -> Point2<S> {
        self.max
    }

    #[inline]
    fn add_margin(&self, margin: Self::Diff) -> Self {
        Aabb2::new(
            Point2::new(self.min.x - margin.x, self.min.y - margin.y),
            Point2::new(self.max.x + margin.x, self.max.y + margin.y),
        )
    }

    #[inline]
    fn transform<T>(&self, transform: &T) -> Self
    where
        T: Transform<Point2<S>>,
    {
        let corners = self.to_corners();
        let transformed_first = transform.transform_point(corners[0]);
        let base = Self::new(transformed_first, transformed_first);
        corners[1..]
            .iter()
            .fold(base, |u, &corner| u.grow(transform.transform_point(corner)))
    }
}

impl<S: BaseNum> fmt::Debug for Aabb2<S> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "[{:?} - {:?}]", self.min, self.max)
    }
}

impl<S: BaseNum> Contains<Point2<S>> for Aabb2<S> {
    #[inline]
    fn contains(&self, p: &Point2<S>) -> bool {
        self.min.x <= p.x && p.x < self.max.x && self.min.y <= p.y && p.y < self.max.y
    }
}

impl<S: BaseNum> Contains<Aabb2<S>> for Aabb2<S> {
    #[inline]
    fn contains(&self, other: &Aabb2<S>) -> bool {
        let other_min = other.min();
        let other_max = other.max();

        other_min.x >= self.min.x
            && other_min.y >= self.min.y
            && other_max.x <= self.max.x
            && other_max.y <= self.max.y
    }
}

impl<S: BaseNum> Contains<Line2<S>> for Aabb2<S> {
    #[inline]
    fn contains(&self, line: &Line2<S>) -> bool {
        self.contains(&line.origin) && self.contains(&line.dest)
    }
}

impl<S: BaseNum> Union for Aabb2<S> {
    type Output = Aabb2<S>;

    fn union(&self, other: &Aabb2<S>) -> Aabb2<S> {
        self.grow(other.min()).grow(other.max())
    }
}

impl<S: BaseNum> SurfaceArea for Aabb2<S> {
    type Scalar = S;

    fn surface_area(&self) -> S {
        self.dim().x * self.dim().y
    }
}

impl<S: BaseFloat> Continuous<Aabb2<S>> for Ray2<S> {
    type Result = Point2<S>;
    fn intersection(&self, aabb: &Aabb2<S>) -> Option<Point2<S>> {
        let ray = self;

        let mut tmin = S::neg_infinity();
        let mut tmax = S::infinity();

        if ray.direction.x != S::zero() {
            let tx1 = (aabb.min.x - ray.origin.x) / ray.direction.x;
            let tx2 = (aabb.max.x - ray.origin.x) / ray.direction.x;
            tmin = tmin.max(tx1.min(tx2));
            tmax = tmax.min(tx1.max(tx2));
        } else if ray.origin.x <= aabb.min.x || ray.origin.x >= aabb.max.x {
            return None;
        }

        if ray.direction.y != S::zero() {
            let ty1 = (aabb.min.y - ray.origin.y) / ray.direction.y;
            let ty2 = (aabb.max.y - ray.origin.y) / ray.direction.y;
            tmin = tmin.max(ty1.min(ty2));
            tmax = tmax.min(ty1.max(ty2));
        } else if ray.origin.y <= aabb.min.y || ray.origin.y >= aabb.max.y {
            return None;
        }

        if (tmin < S::zero() && tmax < S::zero()) || tmax < tmin {
            None
        } else {
            let t = if tmin >= S::zero() { tmin } else { tmax };
            Some(ray.origin + ray.direction * t)
        }
    }
}

impl<S: BaseFloat> Continuous<Ray2<S>> for Aabb2<S> {
    type Result = Point2<S>;

    fn intersection(&self, ray: &Ray2<S>) -> Option<Point2<S>> {
        ray.intersection(self)
    }
}

impl<S: BaseFloat> Discrete<Aabb2<S>> for Ray2<S> {
    fn intersects(&self, aabb: &Aabb2<S>) -> bool {
        let ray = self;

        let mut tmin = S::neg_infinity();
        let mut tmax = S::infinity();

        if ray.direction.x != S::zero() {
            let tx1 = (aabb.min.x - ray.origin.x) / ray.direction.x;
            let tx2 = (aabb.max.x - ray.origin.x) / ray.direction.x;
            tmin = tmin.max(tx1.min(tx2));
            tmax = tmax.min(tx1.max(tx2));
        } else if ray.origin.x <= aabb.min.x || ray.origin.x >= aabb.max.x {
            return false;
        }

        if ray.direction.y != S::zero() {
            let ty1 = (aabb.min.y - ray.origin.y) / ray.direction.y;
            let ty2 = (aabb.max.y - ray.origin.y) / ray.direction.y;
            tmin = tmin.max(ty1.min(ty2));
            tmax = tmax.min(ty1.max(ty2));
        } else if ray.origin.y <= aabb.min.y || ray.origin.y >= aabb.max.y {
            return false;
        }

        tmax >= tmin && (tmin >= S::zero() || tmax >= S::zero())
    }
}

impl<S: BaseFloat> Discrete<Ray2<S>> for Aabb2<S> {
    fn intersects(&self, ray: &Ray2<S>) -> bool {
        ray.intersects(self)
    }
}

impl<S: BaseFloat> Discrete<Aabb2<S>> for Aabb2<S> {
    fn intersects(&self, aabb: &Aabb2<S>) -> bool {
        let (a0, a1) = (self.min(), self.max());
        let (b0, b1) = (aabb.min(), aabb.max());

        a1.x > b0.x && a0.x < b1.x && a1.y > b0.y && a0.y < b1.y
    }
}
