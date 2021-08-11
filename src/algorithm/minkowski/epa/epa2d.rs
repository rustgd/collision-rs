use std::marker;

use cgmath::assert_ulps_ne;
use cgmath::num_traits::NumCast;
use cgmath::prelude::*;
use cgmath::{BaseFloat, Point2, Vector2};

use super::*;
use crate::prelude::*;
use crate::primitive::util::triple_product;
use crate::{CollisionStrategy, Contact};

/// EPA algorithm implementation for 2D. Only to be used in [`GJK`](struct.GJK.html).
#[derive(Debug)]
pub struct EPA2<S> {
    m: marker::PhantomData<S>,
    tolerance: S,
    max_iterations: u32,
}

impl<S> EPA for EPA2<S>
where
    S: BaseFloat,
{
    type Point = Point2<S>;

    fn process<SL, SR, TL, TR>(
        &self,
        simplex: &mut Vec<SupportPoint<Point2<S>>>,
        left: &SL,
        left_transform: &TL,
        right: &SR,
        right_transform: &TR,
    ) -> Option<Contact<Point2<S>>>
    where
        SL: Primitive<Point = Self::Point>,
        SR: Primitive<Point = Self::Point>,
        TL: Transform<Self::Point>,
        TR: Transform<Self::Point>,
    {
        let mut e = match closest_edge(simplex) {
            None => return None,
            Some(e) => e,
        };

        for _ in 0..self.max_iterations {
            let p = SupportPoint::from_minkowski(
                left,
                left_transform,
                right,
                right_transform,
                &e.normal,
            );
            let d = p.v.dot(e.normal);
            if d - e.distance < self.tolerance {
                break;
            } else {
                simplex.insert(e.index, p);
            }
            e = closest_edge(simplex).unwrap();
        }

        Some(Contact::new_with_point(
            CollisionStrategy::FullResolution,
            e.normal,
            e.distance,
            point(simplex, &e),
        ))
    }

    fn new() -> Self {
        Self::new_with_tolerance(NumCast::from(EPA_TOLERANCE).unwrap(), MAX_ITERATIONS)
    }

    fn new_with_tolerance(
        tolerance: <Self::Point as EuclideanSpace>::Scalar,
        max_iterations: u32,
    ) -> Self {
        Self {
            m: marker::PhantomData,
            tolerance,
            max_iterations,
        }
    }
}

/// This function returns the contact point in world space coordinates on shape A.
///
/// Compute the closest point to the origin on the given simplex edge, then use that to interpolate
/// the support points coming from the A shape.
fn point<S>(simplex: &[SupportPoint<Point2<S>>], edge: &Edge<S>) -> Point2<S>
where
    S: BaseFloat,
{
    let b = &simplex[edge.index];
    let a = if edge.index == 0 {
        &simplex[simplex.len() - 1]
    } else {
        &simplex[edge.index - 1]
    };
    let oa = -a.v;
    let ab = b.v - a.v;
    let t = oa.dot(ab) / ab.magnitude2();
    if t < S::zero() {
        a.sup_a
    } else if t > S::one() {
        b.sup_a
    } else {
        a.sup_a + (b.sup_a - a.sup_a) * t
    }
}

#[derive(Debug, PartialEq)]
struct Edge<S> {
    pub normal: Vector2<S>,
    pub distance: S,
    pub index: usize,
}

impl<S> Edge<S>
where
    S: BaseFloat,
{
    pub fn new(normal: Vector2<S>, distance: S, index: usize) -> Self {
        Self {
            normal,
            distance,
            index,
        }
    }
}

fn closest_edge<S>(simplex: &[SupportPoint<Point2<S>>]) -> Option<Edge<S>>
where
    S: BaseFloat,
{
    if simplex.len() < 3 {
        None
    } else {
        let mut edge = Edge::new(Vector2::zero(), S::infinity(), 0);
        for i in 0..simplex.len() {
            let j = if i + 1 == simplex.len() { 0 } else { i + 1 };
            let a = simplex[i].v;
            let b = simplex[j].v;
            let e = b - a;
            let oa = a;
            let n = triple_product(&e, &oa, &e).normalize();
            let d = n.dot(a);
            if d < edge.distance {
                edge = Edge::new(n, d, j);
            }
        }
        assert_ulps_ne!(S::infinity(), edge.distance);
        Some(edge)
    }
}

#[cfg(test)]
mod tests {
    use cgmath::assert_ulps_eq;
    use cgmath::{Basis2, Decomposed, Point2, Rad, Rotation2, Vector2};

    use super::*;
    use crate::algorithm::minkowski::SupportPoint;
    use crate::primitive::*;

    #[test]
    fn test_closest_edge_0() {
        assert_eq!(None, closest_edge::<f32>(&[]));
    }

    #[test]
    fn test_closest_edge_1() {
        assert_eq!(None, closest_edge(&[sup(10., 10.)]));
    }

    #[test]
    fn test_closest_edge_2() {
        assert_eq!(None, closest_edge(&[sup(10., 10.), sup(-10., 5.)]));
    }

    #[test]
    fn test_closest_edge_3() {
        let edge = closest_edge(&[sup(10., 10.), sup(-10., 5.), sup(2., -5.)]);
        assert!(edge.is_some());
        let edge = edge.unwrap();
        assert_eq!(2, edge.index);
        assert_ulps_eq!(2.560_737_4, edge.distance);
        assert_ulps_eq!(-0.640_184_4, edge.normal.x);
        assert_ulps_eq!(-0.768_221_3, edge.normal.y);
    }

    #[test]
    fn test_epa_0() {
        let left = Rectangle::new(10., 10.);
        let left_transform = transform(15., 0., 0.);
        let right = Rectangle::new(10., 10.);
        let right_transform = transform(7., 2., 0.);
        assert!(EPA2::new()
            .process(
                &mut vec![],
                &left,
                &left_transform,
                &right,
                &right_transform
            )
            .is_none());
    }

    #[test]
    fn test_epa_1() {
        let left = Rectangle::new(10., 10.);
        let left_transform = transform(15., 0., 0.);
        let right = Rectangle::new(10., 10.);
        let right_transform = transform(7., 2., 0.);
        let mut simplex = vec![sup(-2., 8.)];
        assert!(EPA2::new()
            .process(
                &mut simplex,
                &left,
                &left_transform,
                &right,
                &right_transform
            )
            .is_none());
    }

    #[test]
    fn test_epa_2() {
        let left = Rectangle::new(10., 10.);
        let left_transform = transform(15., 0., 0.);
        let right = Rectangle::new(10., 10.);
        let right_transform = transform(7., 2., 0.);
        let mut simplex = vec![sup(-2., 8.), sup(18., -12.)];
        assert!(EPA2::new()
            .process(
                &mut simplex,
                &left,
                &left_transform,
                &right,
                &right_transform
            )
            .is_none());
    }

    #[test]
    fn test_epa_3() {
        let left = Rectangle::new(10., 10.);
        let left_transform = transform(15., 0., 0.);
        let right = Rectangle::new(10., 10.);
        let right_transform = transform(7., 2., 0.);
        let mut simplex = vec![sup(-2., 8.), sup(18., -12.), sup(-2., -12.)];
        let contact = EPA2::new().process(
            &mut simplex,
            &left,
            &left_transform,
            &right,
            &right_transform,
        );
        assert!(contact.is_some());
        let contact = contact.unwrap();
        assert_eq!(Vector2::new(-1., 0.), contact.normal);
        assert!(2. - contact.penetration_depth <= f32::EPSILON);
    }

    fn sup(x: f32, y: f32) -> SupportPoint<Point2<f32>> {
        let mut s = SupportPoint::new();
        s.v = Vector2::new(x, y);
        s
    }

    fn transform(x: f32, y: f32, angle: f32) -> Decomposed<Vector2<f32>, Basis2<f32>> {
        Decomposed {
            disp: Vector2::new(x, y),
            rot: Rotation2::from_angle(Rad(angle)),
            scale: 1.,
        }
    }
}
