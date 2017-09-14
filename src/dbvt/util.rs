//! Utilities for use with
//! [`DynamicBoundingVolumeTree`](struct.DynamicBoundingVolumeTree.html).
//!

use std::fmt::Debug;
use std::ops::{Deref, DerefMut};
use std::marker::PhantomData;

use cgmath::prelude::*;
use cgmath::BaseFloat;

use Ray;
use prelude::*;
use super::{DynamicBoundingVolumeTree, TreeValue, Visitor};

/// Trait used to simplify integration with the tree. Instead of keeping track of the node index,
/// fat factor and fat bounds, we provide a wrapper type, together with this trait.
pub trait ValueAabbWrapped: Clone + Debug {
    /// Bounding volume type
    type Bound: Aabb + Clone + Debug;

    /// Borrow the bounding volume
    fn bound(&self) -> &Self::Bound;
}

/// Provides an easier to use wrapper type for use with the DBVT.
///
/// # Examples
///
/// ```
/// # extern crate collision;
/// # extern crate cgmath;
///
/// use cgmath::{Point2, Vector2};
/// use cgmath::prelude::*;
/// use collision::{Aabb2, Ray2};
/// use collision::dbvt::{DynamicBoundingVolumeTree, ValueAabbWrapped,
///                       ValueAabbWrapper, query_ray_closest};
///
/// #[derive(Debug, Clone)]
/// struct Value {
///     id: u32,
///     bound: Aabb2<f32>,
/// }
///
/// impl Value {
///     fn new(id: u32, bound: Aabb2<f32>) -> Self {
///         Self { id, bound }
///     }
/// }
///
/// impl ValueAabbWrapped for Value {
///     type Bound = Aabb2<f32>;
///
///     fn bound(&self) -> &Self::Bound {
///         &self.bound
///     }
/// }
///
/// fn main() {
///     let mut tree = DynamicBoundingVolumeTree::<ValueAabbWrapper<Value>>::new();
///     tree.insert(Value::new(12, aabb2(0., 0., 10., 10.)).into());
///     tree.insert(Value::new(33, aabb2(15., 3., 3., 3.)).into());
///     tree.insert(Value::new(13, aabb2(-145., 34., 2., 2.)).into());
///     tree.insert(Value::new(66, aabb2(123., -10., 10., 10.)).into());
///     tree.insert(Value::new(1, aabb2(22., 50., 16., 16.)).into());
///     tree.insert(Value::new(76, aabb2(7., 3., 1., 1.)).into());
///     tree.insert(Value::new(99, aabb2(19., -12., 3., 3.)).into());
///     tree.do_refit();
///
///     let result = query_ray_closest(
///         &tree,
///         Ray2::new(Point2::new(12., 12.), Vector2::new(0.5, -0.5).normalize()),
///     );
///     assert!(result.is_some());
///     let (v, p) = result.unwrap();
///     assert_eq!(33, v.id);
///     assert_eq!(Point2::new(18., 5.9999995), p);
/// }
///
/// fn aabb2(minx: f32, miny: f32, width: f32, height: f32) -> Aabb2<f32> {
///     Aabb2::new(
///         Point2::new(minx, miny),
///         Point2::new(minx + width, miny + height),
///     )
/// }
/// ```
///
#[derive(Clone, Debug)]
pub struct ValueAabbWrapper<T>
where
    T: ValueAabbWrapped,
    <T::Bound as Aabb>::Diff: Clone + Debug,
{
    index: usize,

    /// Fat factor, defaults to zero, can be changed. After changing, it will be used the next time
    /// the contained value's bounding volumes moves outside the current fat bound in the tree.
    pub fat_factor: <T::Bound as Aabb>::Diff,

    /// The value itself
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
    <T::Bound as Aabb>::Diff: Clone + Debug + VectorSpace,
{
    type Bound = T::Bound;

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

pub struct RayClosestVisitor<S, P, T>
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
        + Debug
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
            },
            None => None,
        }

    }
}

/// Query the given tree for the closest value that intersects the given ray.
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
        + Debug
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
            saved = Some((value, point.clone()));
        }
    }
    saved
}
