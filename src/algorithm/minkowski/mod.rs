//! Algorithms using the Minkowski Sum/Difference

pub use self::epa::{EPA, EPA2, EPA3};
pub use self::gjk::{SimplexProcessor, GJK, GJK2, GJK3};

use std::ops::{Neg, Sub};

use crate::prelude::*;
use cgmath::prelude::*;

mod epa;
mod gjk;

/// Minkowski Sum/Difference support point
#[derive(Clone, Debug, Copy)]
pub struct SupportPoint<P>
where
    P: EuclideanSpace,
{
    v: P::Diff,
    sup_a: P,
    sup_b: P,
}

impl<P> SupportPoint<P>
where
    P: EuclideanSpace,
{
    /// Create a new support point at origin
    pub fn new() -> Self {
        Self {
            v: P::Diff::zero(),
            sup_a: P::origin(),
            sup_b: P::origin(),
        }
    }

    /// Create a new support point from the minkowski difference, using primitive support functions
    pub fn from_minkowski<SL, SR, TL, TR>(
        left: &SL,
        left_transform: &TL,
        right: &SR,
        right_transform: &TR,
        direction: &P::Diff,
    ) -> Self
    where
        SL: Primitive<Point = P>,
        SR: Primitive<Point = P>,
        P::Diff: Neg<Output = P::Diff>,
        TL: Transform<P>,
        TR: Transform<P>,
    {
        let l = left.support_point(direction, left_transform);
        let r = right.support_point(&direction.neg(), right_transform);
        Self {
            v: l - r,
            sup_a: l,
            sup_b: r,
        }
    }
}

impl<P> Sub<P> for SupportPoint<P>
where
    P: EuclideanSpace,
{
    type Output = Self;

    fn sub(self, rhs: P) -> Self {
        SupportPoint {
            v: self.v - rhs.to_vec(),
            sup_a: self.sup_a,
            sup_b: self.sup_b,
        }
    }
}

#[cfg(test)]
mod tests {
    use cgmath::{Basis2, Decomposed, Rad, Rotation2, Vector2};

    use super::SupportPoint;
    use crate::primitive::*;

    fn transform(x: f32, y: f32, angle: f32) -> Decomposed<Vector2<f32>, Basis2<f32>> {
        Decomposed {
            disp: Vector2::new(x, y),
            rot: Rotation2::from_angle(Rad(angle)),
            scale: 1.,
        }
    }

    #[test]
    fn test_support() {
        let left = Rectangle::new(10., 10.);
        let left_transform = transform(15., 0., 0.);
        let right = Rectangle::new(10., 10.);
        let right_transform = transform(-15., 0., 0.);
        let direction = Vector2::new(1., 0.);
        assert_eq!(
            Vector2::new(40., 0.),
            SupportPoint::from_minkowski(
                &left,
                &left_transform,
                &right,
                &right_transform,
                &direction,
            )
            .v
        );
    }
}
