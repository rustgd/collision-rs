//! Utility functions for use with
//! [`DynamicBoundingVolumeTree`](struct.DynamicBoundingVolumeTree.html).
//!

use std::fmt::Debug;
use std::ops::{Deref, DerefMut};

use cgmath::prelude::*;
use cgmath::BaseFloat;

use Ray;
use prelude::*;
use super::{DynamicBoundingVolumeTree, TreeValue};
use super::visitor::ContinuousVisitor;

pub trait ValueAabbWrapped: Clone + Debug {
    type Bound: Aabb + Clone + Debug;

    fn bound(&self) -> &Self::Bound;
}

#[derive(Clone, Debug)]
pub struct ValueAabbWrapper<T>
where
    T: ValueAabbWrapped,
    <T::Bound as Aabb>::Diff: Clone + Debug,
{
    index: usize,
    pub fat_factor: <T::Bound as Aabb>::Diff,
    pub value: T,
}

impl<T> Deref for ValueAabbWrapper<T>
where
    T: ValueAabbWrapped,
    <T::Bound as Aabb>::Diff: Clone + Debug,
{
    type Target = T;

    fn deref(&self) -> &Self::Target {
        &self.value
    }
}

impl<T> DerefMut for ValueAabbWrapper<T>
where
    T: ValueAabbWrapped,
    <T::Bound as Aabb>::Diff: Clone + Debug,
{
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.value
    }
}

impl<T> ValueAabbWrapper<T>
where
    T: ValueAabbWrapped,
    <T::Bound as Aabb>::Diff: Clone + Debug,
{
    pub fn new_impl(value: T, fat_factor: <T::Bound as Aabb>::Diff) -> Self {
        Self {
            index: 0,
            fat_factor,
            value,
        }
    }

    pub fn new(value: T) -> Self {
        Self::new_impl(value, <T::Bound as Aabb>::Diff::zero())
    }
}

impl<T> TreeValue for ValueAabbWrapper<T>
where
    T: ValueAabbWrapped,
    <T::Bound as Aabb>::Diff: Clone + Debug,
{
    type Bound = T::Bound;
    type Vector = <T::Bound as Aabb>::Diff;

    fn bound(&self) -> &Self::Bound {
        self.value.bound()
    }

    fn fat_bound(&self) -> Self::Bound {
        self.value.bound().add_margin(self.fat_factor)
    }

    fn set_index(&mut self, index: usize) {
        self.index = index;
    }

    fn index(&self) -> usize {
        self.index
    }
}

impl<T> From<T> for ValueAabbWrapper<T>
where
    T: ValueAabbWrapped,
    <T::Bound as Aabb>::Diff: Clone + Debug,
{
    fn from(value: T) -> Self {
        Self::new(value)
    }
}

/// Query the given tree for the closest value that intersects the given ray.
pub fn query_ray_closest<'a, S, T: 'a, P>(
    tree: &'a DynamicBoundingVolumeTree<T>,
    ray: &Ray<S, P, P::Diff>,
) -> Option<(&'a T, P)>
where
    S: BaseFloat,
    T: TreeValue,
    P: EuclideanSpace<Scalar = S>,
    P::Diff: VectorSpace<Scalar = S> + InnerSpace,
    T::Bound: Clone
        + Debug
        + Contains<T::Bound>
        + SurfaceArea<Scalar = S>
        + Union<T::Bound, Output = T::Bound>
        + Continuous<Ray<S, P, P::Diff>, Result = P>
        + Discrete<Ray<S, P, P::Diff>>,
{
    let mut saved = None;
    let mut tmin = S::infinity();
    let visitor = ContinuousVisitor::<Ray<S, P, P::Diff>, T>::new(&ray);
    for (value, point) in tree.query(&visitor) {
        let offset = point - ray.origin;
        let t = offset.dot(ray.direction);
        if t < tmin {
            tmin = t;
            saved = Some((value, point.clone()));
        }
    }
    saved
}

#[cfg(test)]
mod tests {

    use cgmath::{Point2, Vector2};

    use super::*;
    use super::super::*;
    use {Aabb2, Ray2};

    #[derive(Debug, Clone)]
    struct Value {
        id: u32,
        bound: Aabb2<f32>,
    }

    impl Value {
        fn new(id: u32, bound: Aabb2<f32>) -> Self {
            Self { id, bound }
        }
    }

    impl ValueAabbWrapped for Value {
        type Bound = Aabb2<f32>;

        fn bound(&self) -> &Self::Bound {
            &self.bound
        }
    }

    #[test]
    fn test() {
        let mut tree = DynamicBoundingVolumeTree::<ValueAabbWrapper<Value>>::new();
        tree.insert(Value::new(12, aabb2(0., 0., 10., 10.)).into());
        tree.insert(Value::new(33, aabb2(15., 3., 3., 3.)).into());
        tree.insert(Value::new(13, aabb2(-145., 34., 2., 2.)).into());
        tree.insert(Value::new(66, aabb2(123., -10., 10., 10.)).into());
        tree.insert(Value::new(1, aabb2(22., 50., 16., 16.)).into());
        tree.insert(Value::new(76, aabb2(7., 3., 1., 1.)).into());
        tree.insert(Value::new(99, aabb2(19., -12., 3., 3.)).into());
        tree.do_refit();

        let result = query_ray_closest(
            &tree,
            &Ray2::new(
                Point2::new(12., 12.),
                Vector2::new(0.5, -0.5).normalize(),
            ),
        );
        assert!(result.is_some());
        let (v, p) = result.unwrap();
        assert_eq!(33, v.id);
        assert_eq!(Point2::new(18., 5.9999995), p);
    }

    fn aabb2(minx: f32, miny: f32, width: f32, height: f32) -> Aabb2<f32> {
        Aabb2::new(
            Point2::new(minx, miny),
            Point2::new(minx + width, miny + height),
        )
    }
}
