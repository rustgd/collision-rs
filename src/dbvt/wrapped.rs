use std::fmt::Debug;

use cgmath::{EuclideanSpace, Zero};

use super::TreeValue;
use {BoundingVolume, HasBound};

/// Value together with bounding volume, for use with DBVT.
#[derive(Debug, Clone)]
pub struct TreeValueWrapped<V, B>
where
    B: BoundingVolume,
    <B::Point as EuclideanSpace>::Diff: Debug,
{
    /// The value
    pub value: V,

    /// The bounding volume
    pub bound: B,

    fat_factor: <B::Point as EuclideanSpace>::Diff,
}

impl<V, B> TreeValueWrapped<V, B>
where
    B: BoundingVolume,
    <B::Point as EuclideanSpace>::Diff: Debug,
{
    /// Create a new shape
    pub fn new(
        value: V,
        bound: &B,
        fat_factor: <B::Point as EuclideanSpace>::Diff,
    ) -> Self
    where
        B: Clone,
    {
        Self {
            value,
            bound: bound.clone(),
            fat_factor,
        }
    }
}

impl<V, B> TreeValue for TreeValueWrapped<V, B>
where
    V: Clone,
    B: BoundingVolume + Clone,
    <B::Point as EuclideanSpace>::Diff: Debug,
{
    type Bound = B;

    fn bound(&self) -> &Self::Bound {
        &self.bound
    }

    fn fat_bound(&self) -> Self::Bound {
        self.bound.with_margin(self.fat_factor)
    }
}

impl<V, B> HasBound for TreeValueWrapped<V, B>
where
    B: BoundingVolume,
    <B::Point as EuclideanSpace>::Diff: Debug,
{
    type Bound = B;

    fn get_bound(&self) -> &Self::Bound {
        &self.bound
    }
}

impl<'a, V, H> From<(V, &'a H)> for TreeValueWrapped<V, H::Bound>
where
    H: HasBound,
    H::Bound: Clone,
    <<H::Bound as BoundingVolume>::Point as EuclideanSpace>::Diff: Debug + Zero,
{
    fn from((value, f): (V, &H)) -> Self {
        Self::new(value, f.get_bound(), <<H::Bound as BoundingVolume>::Point as EuclideanSpace>::Diff::zero())
    }
}