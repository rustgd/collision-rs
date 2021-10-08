//! Axis aligned bounding box for 2D.
//!

use std::fmt;

use cgmath::prelude::*;
use cgmath::{BaseFloat, BaseNum, Point3, Vector3};

use super::{max, min};
use crate::prelude::*;
use crate::{Line3, Plane, Ray3, Sphere};

/// A three-dimensional AABB, aka a rectangular prism.
#[derive(Copy, Clone, PartialEq)]
#[cfg_attr(
    feature = "serde",
    derive(Serialize, Deserialize),
    serde(crate = "serde_crate")
)]
pub struct Aabb3<S> {
    /// Minimum point of the AABB
    pub min: Point3<S>,
    /// Maximum point of the AABB
    pub max: Point3<S>,
}

impl<S: BaseNum> Aabb3<S> {
    /// Construct a new axis-aligned bounding box from two points.
    #[inline]
    pub fn new(p1: Point3<S>, p2: Point3<S>) -> Aabb3<S> {
        Aabb3 {
            min: Point3::new(min(p1.x, p2.x), min(p1.y, p2.y), min(p1.z, p2.z)),
            max: Point3::new(max(p1.x, p2.x), max(p1.y, p2.y), max(p1.z, p2.z)),
        }
    }

    /// Compute corners.
    #[inline]
    pub fn to_corners(&self) -> [Point3<S>; 8] {
        [
            self.min,
            Point3::new(self.max.x, self.min.y, self.min.z),
            Point3::new(self.min.x, self.max.y, self.min.z),
            Point3::new(self.max.x, self.max.y, self.min.z),
            Point3::new(self.min.x, self.min.y, self.max.z),
            Point3::new(self.max.x, self.min.y, self.max.z),
            Point3::new(self.min.x, self.max.y, self.max.z),
            self.max,
        ]
    }
}

impl<S: BaseNum> Aabb for Aabb3<S> {
    type Scalar = S;
    type Diff = Vector3<S>;
    type Point = Point3<S>;

    #[inline]
    fn new(p1: Point3<S>, p2: Point3<S>) -> Aabb3<S> {
        Aabb3::new(p1, p2)
    }

    #[inline]
    fn min(&self) -> Point3<S> {
        self.min
    }

    #[inline]
    fn max(&self) -> Point3<S> {
        self.max
    }

    #[inline]
    fn add_margin(&self, margin: Self::Diff) -> Self {
        Aabb3::new(
            Point3::new(
                self.min.x - margin.x,
                self.min.y - margin.y,
                self.min.z - margin.z,
            ),
            Point3::new(
                self.max.x + margin.x,
                self.max.y + margin.y,
                self.max.z + margin.z,
            ),
        )
    }

    #[inline]
    fn transform<T>(&self, transform: &T) -> Self
    where
        T: Transform<Point3<S>>,
    {
        let corners = self.to_corners();
        let transformed_first = transform.transform_point(corners[0]);
        let base = Self::new(transformed_first, transformed_first);
        corners[1..]
            .iter()
            .fold(base, |u, &corner| u.grow(transform.transform_point(corner)))
    }
}

impl<S: BaseNum> fmt::Debug for Aabb3<S> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "[{:?} - {:?}]", self.min, self.max)
    }
}

impl<S: BaseNum> Contains<Point3<S>> for Aabb3<S> {
    #[inline]
    fn contains(&self, p: &Point3<S>) -> bool {
        self.min.x <= p.x
            && p.x < self.max.x
            && self.min.y <= p.y
            && p.y < self.max.y
            && self.min.z <= p.z
            && p.z < self.max.z
    }
}

impl<S: BaseNum> Contains<Aabb3<S>> for Aabb3<S> {
    #[inline]
    fn contains(&self, other: &Aabb3<S>) -> bool {
        let other_min = other.min();
        let other_max = other.max();

        other_min.x >= self.min.x
            && other_min.y >= self.min.y
            && other_min.z >= self.min.z
            && other_max.x <= self.max.x
            && other_max.y <= self.max.y
            && other_max.z <= self.max.z
    }
}

impl<S: BaseFloat> Contains<Sphere<S>> for Aabb3<S> {
    // will return true for border hits on both min and max extents
    #[inline]
    fn contains(&self, sphere: &Sphere<S>) -> bool {
        (sphere.center.x - sphere.radius) >= self.min.x
            && (sphere.center.y - sphere.radius) >= self.min.y
            && (sphere.center.z - sphere.radius) >= self.min.z
            && (sphere.center.x + sphere.radius) <= self.max.x
            && (sphere.center.y + sphere.radius) <= self.max.y
            && (sphere.center.z + sphere.radius) <= self.max.z
    }
}

impl<S: BaseNum> Contains<Line3<S>> for Aabb3<S> {
    #[inline]
    fn contains(&self, line: &Line3<S>) -> bool {
        self.contains(&line.origin) && self.contains(&line.dest)
    }
}

impl<S: BaseNum> Union for Aabb3<S> {
    type Output = Aabb3<S>;

    fn union(&self, other: &Aabb3<S>) -> Aabb3<S> {
        self.grow(other.min()).grow(other.max())
    }
}

impl<S: BaseFloat> Union<Sphere<S>> for Aabb3<S> {
    type Output = Aabb3<S>;

    fn union(&self, sphere: &Sphere<S>) -> Aabb3<S> {
        self.grow(Point3::new(
            sphere.center.x - sphere.radius,
            sphere.center.y - sphere.radius,
            sphere.center.z - sphere.radius,
        ))
        .grow(sphere.center + Vector3::from_value(sphere.radius))
    }
}

impl<S: BaseFloat> Continuous<Aabb3<S>> for Ray3<S> {
    type Result = Point3<S>;

    fn intersection(&self, aabb: &Aabb3<S>) -> Option<Point3<S>> {
        let ray = self;

        let inv_dir = Vector3::new(S::one(), S::one(), S::one()).div_element_wise(ray.direction);

        let mut t1 = (aabb.min.x - ray.origin.x) * inv_dir.x;
        let mut t2 = (aabb.max.x - ray.origin.x) * inv_dir.x;

        let mut tmin = t1.min(t2);
        let mut tmax = t1.max(t2);

        for i in 1..3 {
            t1 = (aabb.min[i] - ray.origin[i]) * inv_dir[i];
            t2 = (aabb.max[i] - ray.origin[i]) * inv_dir[i];

            tmin = tmin.max(t1.min(t2));
            tmax = tmax.min(t1.max(t2));
        }

        if (tmin < S::zero() && tmax < S::zero()) || tmax < tmin {
            None
        } else {
            let t = if tmin >= S::zero() { tmin } else { tmax };
            Some(ray.origin + ray.direction * t)
        }
    }
}

impl<S: BaseFloat> Continuous<Ray3<S>> for Aabb3<S> {
    type Result = Point3<S>;

    fn intersection(&self, ray: &Ray3<S>) -> Option<Point3<S>> {
        ray.intersection(self)
    }
}

impl<S: BaseFloat> Discrete<Aabb3<S>> for Ray3<S> {
    fn intersects(&self, aabb: &Aabb3<S>) -> bool {
        let ray = self;

        let inv_dir = Vector3::new(S::one(), S::one(), S::one()).div_element_wise(ray.direction);

        let mut t1 = (aabb.min.x - ray.origin.x) * inv_dir.x;
        let mut t2 = (aabb.max.x - ray.origin.x) * inv_dir.x;

        let mut tmin = t1.min(t2);
        let mut tmax = t1.max(t2);

        for i in 1..3 {
            t1 = (aabb.min[i] - ray.origin[i]) * inv_dir[i];
            t2 = (aabb.max[i] - ray.origin[i]) * inv_dir[i];

            tmin = tmin.max(t1.min(t2));
            tmax = tmax.min(t1.max(t2));
        }

        tmax >= tmin && (tmin >= S::zero() || tmax >= S::zero())
    }
}

impl<S: BaseFloat> Discrete<Ray3<S>> for Aabb3<S> {
    fn intersects(&self, ray: &Ray3<S>) -> bool {
        ray.intersects(self)
    }
}

impl<S: BaseFloat> Discrete<Aabb3<S>> for Aabb3<S> {
    fn intersects(&self, aabb: &Aabb3<S>) -> bool {
        let (a0, a1) = (self.min(), self.max());
        let (b0, b1) = (aabb.min(), aabb.max());

        a1.x > b0.x && a0.x < b1.x && a1.y > b0.y && a0.y < b1.y && a1.z > b0.z && a0.z < b1.z
    }
}

impl<S: BaseFloat> PlaneBound<S> for Aabb3<S> {
    fn relate_plane(&self, plane: Plane<S>) -> Relation {
        let corners = self.to_corners();
        let first = corners[0].relate_plane(plane);
        for p in corners[1..].iter() {
            if p.relate_plane(plane) != first {
                return Relation::Cross;
            }
        }
        first
    }
}

impl<S: BaseNum> SurfaceArea for Aabb3<S> {
    type Scalar = S;

    fn surface_area(&self) -> S {
        let dim = self.dim();
        let two = S::one() + S::one();
        two * ((dim.x * dim.y) + (dim.x * dim.z) + (dim.y * dim.z))
    }
}
