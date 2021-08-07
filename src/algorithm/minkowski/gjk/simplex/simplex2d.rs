use std::marker;
use std::ops::Neg;

use cgmath::prelude::*;
use cgmath::ulps_eq;
use cgmath::{BaseFloat, Point2, Vector2};

use super::{Simplex, SimplexProcessor};
use crate::primitive::util::{get_closest_point_on_edge, triple_product};

/// Simplex processor implementation for 2D. Only to be used in [`GJK`](struct.GJK.html).
#[derive(Debug)]
pub struct SimplexProcessor2<S> {
    m: marker::PhantomData<S>,
}

impl<S> SimplexProcessor for SimplexProcessor2<S>
where
    S: BaseFloat,
{
    type Point = Point2<S>;

    fn reduce_to_closest_feature(
        &self,
        simplex: &mut Simplex<Point2<S>>,
        d: &mut Vector2<S>,
    ) -> bool {
        // 3 points
        if simplex.len() == 3 {
            let a = simplex[2].v;
            let b = simplex[1].v;
            let c = simplex[0].v;

            let ao = a.neg();
            let ab = b - a;
            let ac = c - a;

            let abp = triple_product(&ac, &ab, &ab);
            if abp.dot(ao) > S::zero() {
                simplex.remove(0);
                *d = abp;
            } else {
                let acp = triple_product(&ab, &ac, &ac);
                if acp.dot(ao) > S::zero() {
                    simplex.remove(1);
                    *d = acp;
                } else {
                    return true;
                }
            }
        }
        // 2 points
        else if simplex.len() == 2 {
            let a = simplex[1].v;
            let b = simplex[0].v;

            let ao = a.neg();
            let ab = b - a;

            *d = triple_product(&ab, &ao, &ab);
            if ulps_eq!(*d, Vector2::zero()) {
                *d = Vector2::new(-ab.y, ab.x);
            }
        }
        // 0-1 point means we can't really do anything
        false
    }

    /// Get the closest point on the simplex to the origin.
    ///
    /// Make simplex only retain the closest feature to the origin.
    fn get_closest_point_to_origin(&self, simplex: &mut Simplex<Point2<S>>) -> Vector2<S> {
        let mut d = Vector2::zero();

        // reduce simplex to the closest feature to the origin
        // if check_origin return true, the origin is inside the simplex, so return the zero vector
        // if not, the simplex will be the closest edge to the origin, and d the normal of the edge
        // in the direction of the origin
        if self.reduce_to_closest_feature(simplex, &mut d) {
            return d;
        }

        // compute closest point to origin on the simplex (which is now an edge)
        if simplex.len() == 1 {
            simplex[0].v
        } else {
            get_closest_point_on_edge(&simplex[1].v, &simplex[0].v, &Vector2::zero())
        }
    }

    fn new() -> Self {
        Self {
            m: marker::PhantomData,
        }
    }
}

#[cfg(test)]
mod tests {
    use cgmath::Vector2;

    use super::*;
    use crate::algorithm::minkowski::SupportPoint;
    use cgmath::assert_ulps_eq;
    use smallvec::smallvec;

    #[test]
    fn test_check_origin_empty() {
        let processor = SimplexProcessor2::new();
        let mut direction = Vector2::new(1., 0.);
        let mut simplex = smallvec![];
        assert!(!processor.reduce_to_closest_feature(&mut simplex, &mut direction));
        assert_eq!(0, simplex.len());
        assert_eq!(Vector2::new(1., 0.), direction);
    }

    #[test]
    fn test_check_origin_single() {
        let processor = SimplexProcessor2::new();
        let mut direction = Vector2::new(1., 0.);
        let mut simplex = smallvec![sup(40., 0.)];
        assert!(!processor.reduce_to_closest_feature(&mut simplex, &mut direction));
        assert_eq!(1, simplex.len());
        assert_eq!(Vector2::new(1., 0.), direction);
    }

    #[test]
    fn test_check_origin_edge() {
        let processor = SimplexProcessor2::new();
        let mut direction = Vector2::new(1., 0.);
        let mut simplex = smallvec![sup(40., 10.), sup(-10., 10.)];
        assert!(!processor.reduce_to_closest_feature(&mut simplex, &mut direction));
        assert_eq!(2, simplex.len());
        assert!(0. - direction.x <= f32::EPSILON);
        assert!(direction.y < 0.);
    }

    #[test]
    fn test_check_origin_triangle_outside_ac() {
        let processor = SimplexProcessor2::new();
        let mut direction = Vector2::new(1., 0.);
        let mut simplex = smallvec![sup(40., 10.), sup(-10., 10.), sup(0., 3.)];
        assert!(!processor.reduce_to_closest_feature(&mut simplex, &mut direction));
        assert_eq!(2, simplex.len());
        assert!(direction.x < 0.);
        assert!(direction.y < 0.);
    }

    #[test]
    fn test_check_origin_triangle_outside_ab() {
        let processor = SimplexProcessor2::new();
        let mut direction = Vector2::new(1., 0.);
        let mut simplex = smallvec![sup(40., 10.), sup(10., 10.), sup(3., -3.)];
        assert!(!processor.reduce_to_closest_feature(&mut simplex, &mut direction));
        assert_eq!(2, simplex.len());
        assert!(direction.x < 0.);
        assert!(direction.y > 0.);
    }

    #[test]
    fn test_check_origin_triangle_hit() {
        let processor = SimplexProcessor2::new();
        let mut direction = Vector2::new(1., 0.);
        let mut simplex = smallvec![sup(40., 10.), sup(-10., 10.), sup(0., -3.)];
        assert!(processor.reduce_to_closest_feature(&mut simplex, &mut direction));
        assert_eq!(3, simplex.len());
        assert_eq!(Vector2::new(1., 0.), direction);
    }

    #[test]
    fn test_closest_point_to_origin_triangle() {
        let processor = SimplexProcessor2::new();
        let mut simplex = smallvec![sup(40., 10.), sup(-10., 10.), sup(0., 3.)];
        let p = processor.get_closest_point_to_origin(&mut simplex);
        assert_eq!(2, simplex.len());
        assert_ulps_eq!(Vector2::new(0., 3.), p);
    }

    #[test]
    fn test_closest_point_to_origin_triangle_inside() {
        let processor = SimplexProcessor2::new();
        let mut simplex = smallvec![sup(40., 10.), sup(-10., 10.), sup(0., -3.)];
        let p = processor.get_closest_point_to_origin(&mut simplex);
        assert_eq!(3, simplex.len());
        assert_ulps_eq!(Vector2::new(0., 0.), p);
    }

    #[test]
    fn test_closest_point_to_origin_edge() {
        let processor = SimplexProcessor2::new();
        let mut simplex = smallvec![sup(40., 10.), sup(-10., 10.)];
        let p = processor.get_closest_point_to_origin(&mut simplex);
        assert_eq!(2, simplex.len());
        assert_ulps_eq!(Vector2::new(0., 10.), p);
    }

    fn sup(x: f32, y: f32) -> SupportPoint<Point2<f32>> {
        let mut s = SupportPoint::new();
        s.v = Vector2::new(x, y);
        s
    }
}
