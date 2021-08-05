//! Concrete visitor implementations for use with
//! [`query`](struct.DynamicBoundingVolumeTree.html#method.query).
//!

use std::marker::PhantomData;

use cgmath::BaseFloat;

use super::{TreeValue, Visitor};
use crate::prelude::*;
use crate::{Frustum, PlaneBound, Relation};

/// Visitor for doing continuous intersection testing on the DBVT.
///
/// Will return the result from the
/// [`Continuous`](../trait.Continuous.html) implementation
/// of bound.intersection(self.bound).
///
#[derive(Debug)]
pub struct ContinuousVisitor<'a, B, T> {
    bound: &'a B,
    marker: PhantomData<T>,
}

impl<'a, B: 'a, T> ContinuousVisitor<'a, B, T>
where
    T: TreeValue,
    T::Bound: Continuous<B> + Discrete<B>,
{
    /// Create a new visitor that will do continuous intersection tests using the given bound.
    ///
    pub fn new(bound: &'a B) -> Self {
        Self {
            bound,
            marker: PhantomData,
        }
    }
}

impl<'a, B: 'a, T> Visitor for ContinuousVisitor<'a, B, T>
where
    T: TreeValue,
    T::Bound: Continuous<B> + Discrete<B>,
{
    type Bound = T::Bound;
    type Result = <T::Bound as Continuous<B>>::Result;

    fn accept(&mut self, bound: &Self::Bound, _: bool) -> Option<Self::Result> {
        bound.intersection(self.bound)
    }
}

/// Visitor for doing discrete intersection testing on the DBVT.
///
/// Will return () for intersections with the
/// [`Discrete`](../trait.Discrete.html) implementation
/// of bound.intersects(self.bound).
///
#[derive(Debug)]
pub struct DiscreteVisitor<'a, B, T> {
    bound: &'a B,
    marker: PhantomData<T>,
}

impl<'a, B: 'a, T> DiscreteVisitor<'a, B, T>
where
    T: TreeValue,
    T::Bound: Discrete<B>,
{
    /// Create a new visitor that will do discrete intersection tests using the given bound.
    pub fn new(bound: &'a B) -> Self {
        Self {
            bound,
            marker: PhantomData,
        }
    }
}

impl<'a, B: 'a, T> Visitor for DiscreteVisitor<'a, B, T>
where
    T: TreeValue,
    T::Bound: Discrete<B>,
{
    type Bound = T::Bound;
    type Result = ();

    fn accept(&mut self, bound: &Self::Bound, _: bool) -> Option<()> {
        if bound.intersects(self.bound) {
            Some(())
        } else {
            None
        }
    }
}

/// Visitor for doing frustum intersection testing on the DBVT.
///
/// Will return the relation for intersections with the
/// [`Bound`](../trait.Bound.html) implementation
/// of self.frustum.contains(bound).
///
#[derive(Debug)]
pub struct FrustumVisitor<'a, S, T>
where
    S: BaseFloat,
{
    frustum: &'a Frustum<S>,
    marker: PhantomData<T>,
}

impl<'a, S, T> FrustumVisitor<'a, S, T>
where
    S: BaseFloat,
    T: TreeValue,
    T::Bound: PlaneBound<S>,
{
    /// Create a new visitor that will do containment tests using the given frustum
    pub fn new(frustum: &'a Frustum<S>) -> Self {
        Self {
            frustum,
            marker: PhantomData,
        }
    }
}

impl<'a, S, T> Visitor for FrustumVisitor<'a, S, T>
where
    S: BaseFloat,
    T: TreeValue,
    T::Bound: PlaneBound<S>,
{
    type Bound = T::Bound;
    type Result = Relation;

    fn accept(&mut self, bound: &Self::Bound, _: bool) -> Option<Relation> {
        let r = self.frustum.contains(bound);
        if r == Relation::Out {
            None
        } else {
            Some(r)
        }
    }
}
