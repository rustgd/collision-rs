//! Utilities for use with
//! [`DynamicBoundingVolumeTree`](struct.DynamicBoundingVolumeTree.html).
//!

use std::marker::PhantomData;

use cgmath::prelude::*;
use cgmath::BaseFloat;

use super::{ContinuousVisitor, DynamicBoundingVolumeTree, TreeValue, Visitor};
use crate::prelude::*;
use crate::Ray;

struct RayClosestVisitor<S, P, T>
where
    S: BaseFloat,
    T: TreeValue,
    P: EuclideanSpace<Scalar = S>,
{
    ray: Ray<S, P, P::Diff>,
    min: S,
    marker: PhantomData<T>,
}

impl<S, P, T> RayClosestVisitor<S, P, T>
where
    S: BaseFloat,
    T: TreeValue,
    P: EuclideanSpace<Scalar = S>,
{
    pub fn new(ray: Ray<S, P, P::Diff>) -> Self {
        Self {
            ray,
            min: S::infinity(),
            marker: PhantomData,
        }
    }
}

impl<S, P, T> Visitor for RayClosestVisitor<S, P, T>
where
    S: BaseFloat,
    T: TreeValue,
    P: EuclideanSpace<Scalar = S>,
    P::Diff: VectorSpace<Scalar = S> + InnerSpace,
    T::Bound: Clone
        + Contains<T::Bound>
        + SurfaceArea<Scalar = S>
        + Union<T::Bound, Output = T::Bound>
        + Continuous<Ray<S, P, P::Diff>, Result = P>,
{
    type Bound = T::Bound;
    type Result = P;

    fn accept(&mut self, bound: &Self::Bound, is_leaf: bool) -> Option<Self::Result> {
        match bound.intersection(&self.ray) {
            Some(point) => {
                let offset = point - self.ray.origin;
                let t = offset.dot(self.ray.direction);
                if t < self.min {
                    if is_leaf {
                        self.min = t;
                    }
                    Some(point)
                } else {
                    None
                }
            }
            None => None,
        }
    }
}

/// Query the given tree for the closest value that intersects the given ray.
///
/// ### Parameters:
///
/// - `tree`: DBVT to query.
/// - `ray`: Ray to find the closest intersection for.
///
/// ### Returns
///
/// Optionally returns the value that had the closest intersection with the ray, along with the
/// actual intersection point.
///
pub fn query_ray_closest<'a, S, T: 'a, P>(
    tree: &'a DynamicBoundingVolumeTree<T>,
    ray: Ray<S, P, P::Diff>,
) -> Option<(&'a T, P)>
where
    S: BaseFloat,
    T: TreeValue,
    P: EuclideanSpace<Scalar = S>,
    P::Diff: VectorSpace<Scalar = S> + InnerSpace,
    T::Bound: Clone
        + Contains<T::Bound>
        + SurfaceArea<Scalar = S>
        + Union<T::Bound, Output = T::Bound>
        + Continuous<Ray<S, P, P::Diff>, Result = P>
        + Discrete<Ray<S, P, P::Diff>>,
{
    let mut saved = None;
    let mut tmin = S::infinity();
    let mut visitor = RayClosestVisitor::<S, P, T>::new(ray);
    for (value, point) in tree.query(&mut visitor) {
        let offset = point - ray.origin;
        let t = offset.dot(ray.direction);
        if t < tmin {
            tmin = t;
            saved = Some((value, point));
        }
    }
    saved
}

/// Query the given tree for all values that intersects the given ray.
///
/// ### Parameters:
///
/// - `tree`: DBVT to query.
/// - `ray`: Ray to find intersections for.
///
/// ### Returns
///
/// Returns all values that intersected the ray, and also at which point the intersections occurred.
pub fn query_ray<'a, S, T: 'a, P>(
    tree: &'a DynamicBoundingVolumeTree<T>,
    ray: Ray<S, P, P::Diff>,
) -> Vec<(&'a T, P)>
where
    S: BaseFloat,
    T: TreeValue,
    P: EuclideanSpace<Scalar = S>,
    P::Diff: VectorSpace<Scalar = S> + InnerSpace,
    T::Bound: Clone
        + Contains<T::Bound>
        + SurfaceArea<Scalar = S>
        + Union<T::Bound, Output = T::Bound>
        + Continuous<Ray<S, P, P::Diff>, Result = P>
        + Discrete<Ray<S, P, P::Diff>>,
{
    let mut visitor = ContinuousVisitor::<_, T>::new(&ray);
    tree.query(&mut visitor)
}
