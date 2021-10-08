//! Particle primitive

use std::marker;
use std::ops::Range;

use cgmath::prelude::*;
use cgmath::{BaseFloat, Point2, Point3};

use crate::prelude::*;
use crate::Ray;

/// Represents a particle in space.
///
/// Only have implementations of intersection tests for movement,
/// using `(Particle, Range<P>)` where `P` is the positional `Point` of the particle at the start
/// and end of movement.
/// These intersection tests can be used with any primitive that has `Ray` intersection tests
/// implemented.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(
    feature = "serde",
    derive(Serialize, Deserialize),
    serde(crate = "serde_crate")
)]
pub struct Particle<P> {
    m: marker::PhantomData<P>,
}

impl<P> Particle<P> {
    /// Create a new particle
    pub fn new() -> Self {
        Self {
            m: marker::PhantomData,
        }
    }
}

/// 2D particle
pub type Particle2<S> = Particle<Point2<S>>;

/// 3D particle
pub type Particle3<S> = Particle<Point3<S>>;

impl<P, C> Discrete<(Particle<P>, Range<P>)> for C
where
    C: Continuous<Ray<P::Scalar, P, P::Diff>, Result = P>,
    P: EuclideanSpace,
    P::Diff: InnerSpace,
    P::Scalar: BaseFloat,
{
    fn intersects(&self, &(_, ref range): &(Particle<P>, Range<P>)) -> bool {
        let direction = range.end - range.start;
        let ray = Ray::new(range.start, direction.normalize());
        match self.intersection(&ray) {
            None => false,
            Some(p) => (p - range.start).magnitude2() <= direction.magnitude2(),
        }
    }
}

impl<P> Discrete<Ray<P::Scalar, P, P::Diff>> for Particle<P>
where
    P: EuclideanSpace,
    P::Diff: InnerSpace,
    P::Scalar: BaseFloat,
{
    /// Ray needs to be in particle object space
    fn intersects(&self, ray: &Ray<P::Scalar, P, P::Diff>) -> bool {
        P::origin().intersects(ray)
    }
}

impl<P> Continuous<Ray<P::Scalar, P, P::Diff>> for Particle<P>
where
    P: EuclideanSpace,
    P::Diff: InnerSpace,
    P::Scalar: BaseFloat,
{
    type Result = P;

    /// Ray needs to be in particle object space
    fn intersection(&self, ray: &Ray<P::Scalar, P, P::Diff>) -> Option<P> {
        P::origin().intersection(ray)
    }
}

impl<P, C> DiscreteTransformed<(Particle<P>, Range<P>)> for C
where
    C: ContinuousTransformed<Ray<P::Scalar, P, P::Diff>, Result = P, Point = P>,
    P: EuclideanSpace,
    P::Diff: InnerSpace,
    P::Scalar: BaseFloat,
{
    type Point = P;

    fn intersects_transformed<T>(
        &self,
        &(_, ref range): &(Particle<P>, Range<P>),
        transform: &T,
    ) -> bool
    where
        T: Transform<P>,
    {
        let direction = range.end - range.start;
        let ray = Ray::new(range.start, direction.normalize());
        match self.intersection_transformed(&ray, transform) {
            None => false,
            Some(p) => (p - range.start).magnitude2() <= direction.magnitude2(),
        }
    }
}

impl<P, C> Continuous<(Particle<P>, Range<P>)> for C
where
    C: Continuous<Ray<P::Scalar, P, P::Diff>, Result = P>,
    P: EuclideanSpace,
    P::Diff: InnerSpace,
    P::Scalar: BaseFloat,
{
    type Result = P;

    fn intersection(&self, &(_, ref range): &(Particle<P>, Range<P>)) -> Option<P> {
        let direction = range.end - range.start;
        let ray = Ray::new(range.start, direction.normalize());
        self.intersection(&ray).and_then(|p| {
            if (p - range.start).magnitude2() <= direction.magnitude2() {
                Some(p)
            } else {
                None
            }
        })
    }
}

impl<P, C> ContinuousTransformed<(Particle<P>, Range<P>)> for C
where
    C: ContinuousTransformed<Ray<P::Scalar, P, P::Diff>, Result = P, Point = P>,
    P: EuclideanSpace,
    P::Diff: InnerSpace,
    P::Scalar: BaseFloat,
{
    type Point = P;
    type Result = P;

    fn intersection_transformed<T>(
        &self,
        &(_, ref range): &(Particle<P>, Range<P>),
        transform: &T,
    ) -> Option<P>
    where
        T: Transform<P>,
    {
        let direction = range.end - range.start;
        let ray = Ray::new(range.start, direction.normalize());
        self.intersection_transformed(&ray, transform)
            .and_then(|p| {
                if (p - range.start).magnitude2() <= direction.magnitude2() {
                    Some(p)
                } else {
                    None
                }
            })
    }
}

#[cfg(test)]
mod tests {
    use cgmath::assert_ulps_eq;
    use cgmath::prelude::*;
    use cgmath::{Basis2, Decomposed, Point2, Rad, Vector2};

    use super::*;
    use crate::primitive::Circle;

    #[test]
    fn test_discrete() {
        let circle = Circle::new(4.);
        assert!(circle.intersects(&(Particle::new(), Point2::new(-5., -5.)..Point2::new(5., 5.))));
        assert!(!circle.intersects(&(
            Particle::new(),
            Point2::new(-5., -5.)..Point2::new(-8., -8.)
        )));
    }

    #[test]
    fn test_discrete_transformed() {
        let circle = Circle::new(4.);
        let t = transform(0., 0., 0.);
        assert!(circle.intersects_transformed(
            &(Particle::new(), Point2::new(-5., -5.)..Point2::new(5., 5.)),
            &t
        ));
        assert!(!circle.intersects_transformed(
            &(
                Particle::new(),
                Point2::new(-5., -5.)..Point2::new(-8., -8.)
            ),
            &t
        ));
        let t = transform(10., 10., 0.);
        assert!(!circle.intersects_transformed(
            &(Particle::new(), Point2::new(-5., -5.)..Point2::new(5., 5.)),
            &t
        ));
    }

    #[test]
    fn test_continuous() {
        let circle = Circle::new(4.);
        assert_ulps_eq!(
            Point2::new(-2.828_427_124_746_190_3, -2.828_427_124_746_190_3),
            circle
                .intersection(&(Particle::new(), Point2::new(-5., -5.)..Point2::new(5., 5.)))
                .unwrap()
        );
        assert_eq!(
            None,
            circle.intersection(&(
                Particle::new(),
                Point2::new(-5., -5.)..Point2::new(-8., -8.)
            ))
        );
    }

    #[test]
    fn test_continuous_transformed() {
        let circle = Circle::new(4.);
        let t = transform(0., 0., 0.);
        assert_ulps_eq!(
            Point2::new(-2.828_427_124_746_190_3, -2.828_427_124_746_190_3),
            circle
                .intersection_transformed(
                    &(Particle::new(), Point2::new(-5., -5.)..Point2::new(5., 5.)),
                    &t
                )
                .unwrap()
        );
        assert_eq!(
            None,
            circle.intersection_transformed(
                &(
                    Particle::new(),
                    Point2::new(-5., -5.)..Point2::new(-8., -8.)
                ),
                &t
            )
        );
        let t = transform(10., 10., 0.);
        assert_eq!(
            None,
            circle.intersection_transformed(
                &(Particle::new(), Point2::new(-5., -5.)..Point2::new(5., 5.)),
                &t
            )
        );
    }

    fn transform(dx: f32, dy: f32, rot: f32) -> Decomposed<Vector2<f32>, Basis2<f32>> {
        Decomposed {
            scale: 1.,
            rot: Rotation2::from_angle(Rad(rot)),
            disp: Vector2::new(dx, dy),
        }
    }
}
