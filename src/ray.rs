//! Generic rays

use core::fmt;
use std::marker::PhantomData;

use cgmath::prelude::*;
use cgmath::{BaseFloat, BaseNum};
use cgmath::{Point2, Point3};
use cgmath::{Vector2, Vector3};

use crate::traits::{Continuous, ContinuousTransformed, Discrete, DiscreteTransformed};

/// A generic ray starting at `origin` and extending infinitely in
/// `direction`.
#[derive(Copy, Clone, PartialEq)]
#[cfg_attr(
    feature = "serde",
    derive(Serialize, Deserialize),
    serde(crate = "serde_crate")
)]
pub struct Ray<S, P, V> {
    /// Ray origin
    pub origin: P,
    /// Normalized ray direction
    pub direction: V,
    phantom_s: PhantomData<S>,
}

impl<S, V, P> Ray<S, P, V>
where
    S: BaseNum,
    V: VectorSpace<Scalar = S>,
    P: EuclideanSpace<Scalar = S, Diff = V>,
{
    /// Create a generic ray starting at `origin` and extending infinitely in
    /// `direction`.
    pub fn new(origin: P, direction: V) -> Ray<S, P, V> {
        Ray {
            origin,
            direction,
            phantom_s: PhantomData,
        }
    }

    /// Create a new ray by applying a transform.
    pub fn transform<T>(&self, transform: &T) -> Self
    where
        T: Transform<P>,
    {
        Self::new(
            transform.transform_point(self.origin),
            transform.transform_vector(self.direction),
        )
    }
}

impl<S, P: fmt::Debug, V: fmt::Debug> fmt::Debug for Ray<S, P, V> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_tuple("Ray")
            .field(&self.origin)
            .field(&self.direction)
            .finish()
    }
}

/// 2D ray
pub type Ray2<S> = Ray<S, Point2<S>, Vector2<S>>;

/// 3D ray
pub type Ray3<S> = Ray<S, Point3<S>, Vector3<S>>;

impl<S, P> Continuous<Ray<S, P, P::Diff>> for P
where
    S: BaseFloat,
    P: EuclideanSpace<Scalar = S>,
    P::Diff: InnerSpace<Scalar = S>,
{
    type Result = P;
    fn intersection(&self, ray: &Ray<S, P, P::Diff>) -> Option<P> {
        if self.intersects(ray) {
            Some(*self)
        } else {
            None
        }
    }
}

impl<S, P> Discrete<Ray<S, P, P::Diff>> for P
where
    S: BaseFloat,
    P: EuclideanSpace<Scalar = S>,
    P::Diff: InnerSpace<Scalar = S>,
{
    fn intersects(&self, ray: &Ray<S, P, P::Diff>) -> bool {
        let p = *self;
        let l = p - ray.origin;
        let tca = l.dot(ray.direction);
        tca > S::zero()
            && (tca * tca).relative_eq(
                &l.magnitude2(),
                S::default_epsilon(),
                S::default_max_relative(),
            )
    }
}

impl<P, C> DiscreteTransformed<Ray<P::Scalar, P, P::Diff>> for C
where
    C: Discrete<Ray<P::Scalar, P, P::Diff>>,
    P: EuclideanSpace,
    P::Scalar: BaseFloat,
{
    type Point = P;

    fn intersects_transformed<T>(&self, ray: &Ray<P::Scalar, P, P::Diff>, transform: &T) -> bool
    where
        T: Transform<P>,
    {
        self.intersects(&ray.transform(&transform.inverse_transform().unwrap()))
    }
}

impl<P, C> ContinuousTransformed<Ray<P::Scalar, P, P::Diff>> for C
where
    C: Continuous<Ray<P::Scalar, P, P::Diff>, Result = P>,
    P: EuclideanSpace,
    P::Scalar: BaseFloat,
{
    type Point = P;
    type Result = P;

    fn intersection_transformed<T>(
        &self,
        ray: &Ray<P::Scalar, P, P::Diff>,
        transform: &T,
    ) -> Option<P>
    where
        T: Transform<P>,
    {
        self.intersection(&ray.transform(&transform.inverse_transform().unwrap()))
            .map(|p| transform.transform_point(p))
    }
}
