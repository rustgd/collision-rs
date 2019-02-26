use std::fmt::Debug;

use cgmath::{EuclideanSpace, Zero};

use super::TreeValue;
use crate::{Bound, HasBound};

/// Value together with bounding volume, for use with DBVT.
#[derive(Debug, Clone)]
pub struct TreeValueWrapped<V, B>
where
    B: Bound,
    <B::Point as EuclideanSpace>::Diff: Debug,
{
    /// The value
    pub value: V,

    /// The bounding volume
    pub bound: B,

    margin: <B::Point as EuclideanSpace>::Diff,
}

impl<V, B> TreeValueWrapped<V, B>
where
    B: Bound,
    <B::Point as EuclideanSpace>::Diff: Debug,
{
    /// Create a new shape
    pub fn new(value: V, bound: B, margin: <B::Point as EuclideanSpace>::Diff) -> Self {
        Self {
            value,
            bound,
            margin,
        }
    }
}

impl<V, B> TreeValue for TreeValueWrapped<V, B>
where
    V: Clone,
    B: Bound + Clone,
    <B::Point as EuclideanSpace>::Diff: Debug,
{
    type Bound = B;

    fn bound(&self) -> &Self::Bound {
        &self.bound
    }

    fn get_bound_with_margin(&self) -> Self::Bound {
        self.bound.with_margin(self.margin)
    }
}

impl<V, B> HasBound for TreeValueWrapped<V, B>
where
    B: Bound,
    <B::Point as EuclideanSpace>::Diff: Debug,
{
    type Bound = B;

    fn bound(&self) -> &Self::Bound {
        &self.bound
    }
}

impl<V, B, P> From<(V, B, P::Diff)> for TreeValueWrapped<V, B>
where
    P: EuclideanSpace,
    B: Bound<Point = P> + Clone,
    P::Diff: Debug,
{
    fn from((value, bound, margin): (V, B, P::Diff)) -> Self {
        Self::new(value, bound, margin)
    }
}

impl<V, B> From<(V, B)> for TreeValueWrapped<V, B>
where
    B: Bound + Clone,
    <<B as Bound>::Point as EuclideanSpace>::Diff: Debug + Zero,
{
    fn from((value, bound): (V, B)) -> Self {
        Self::new(
            value,
            bound,
            <<B as Bound>::Point as EuclideanSpace>::Diff::zero(),
        )
    }
}
