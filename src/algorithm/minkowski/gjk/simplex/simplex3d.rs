use std::marker;
use std::ops::Neg;

use cgmath::{BaseFloat, Point3, Vector3};
use cgmath::prelude::*;

use super::SimplexProcessor;
use algorithm::minkowski::SupportPoint;

/// Simplex processor implementation for 3D. Only to be used in [`GJK`](struct.GJK.html).
#[derive(Debug)]
pub struct SimplexProcessor3<S> {
    m: marker::PhantomData<S>,
}

impl<S> SimplexProcessor for SimplexProcessor3<S>
where
    S: BaseFloat,
{
    type Point = Point3<S>;

    fn check_origin(&self, simplex: &mut Vec<SupportPoint<Point3<S>>>, v: &mut Vector3<S>) -> bool {
        // 4 points, full tetrahedron, origin could be inside
        if simplex.len() == 4 {
            let a = simplex[3].v;
            let b = simplex[2].v;
            let c = simplex[1].v;
            let d = simplex[0].v;

            let ao = a.neg();
            let ab = b - a;
            let ac = c - a;

            let abc = ab.cross(ac);

            // origin outside plane ABC, remove D and check side
            // need to check both edges
            if abc.dot(ao) > S::zero() {
                simplex.remove(0);
                check_side(&abc, &ab, &ac, &ao, simplex, v, true, false);
            } else {
                let ad = d - a;
                let acd = ac.cross(ad);
                // origin outside plane ACD, remove B and check side
                // no need to test first edge, since that region is also over ABC
                if acd.dot(ao) > S::zero() {
                    simplex.remove(2);
                    check_side(&acd, &ac, &ad, &ao, simplex, v, true, true);
                } else {
                    let adb = ad.cross(ab);
                    // origin outside plane ADB, remove C and check side
                    // no need to test edges, since those regions are covered in earlier tests
                    if adb.dot(ao) > S::zero() {
                        // [b, d, a]
                        simplex.remove(1);
                        simplex.swap(0, 1);
                        *v = adb;
                    // origin is inside simplex
                    } else {
                        return true;
                    }
                }
            }
        }
        // 3 points, can't do origin check, find closest feature to origin, and move that way
        else if simplex.len() == 3 {
            let a = simplex[2].v;
            let b = simplex[1].v;
            let c = simplex[0].v;

            let ao = a.neg();
            let ab = b - a;
            let ac = c - a;

            check_side(&ab.cross(ac), &ab, &ac, &ao, simplex, v, false, false);
        }
        // 2 points, can't do much with only an edge, only find a new search direction
        else if simplex.len() == 2 {
            let a = simplex[1].v;
            let b = simplex[0].v;

            let ao = a.neg();
            let ab = b - a;

            *v = cross_aba(&ab, &ao);
        }
        // 0-1 points
        false
    }

    fn new() -> Self {
        Self {
            m: marker::PhantomData,
        }
    }
}

#[inline]
fn cross_aba<S>(a: &Vector3<S>, b: &Vector3<S>) -> Vector3<S>
where
    S: BaseFloat,
{
    a.cross(*b).cross(*a)
}

#[inline]
fn check_side<S>(
    abc: &Vector3<S>,
    ab: &Vector3<S>,
    ac: &Vector3<S>,
    ao: &Vector3<S>,
    simplex: &mut Vec<SupportPoint<Point3<S>>>,
    v: &mut Vector3<S>,
    above: bool,
    ignore_ab: bool,
) where
    S: BaseFloat,
{
    let ab_perp = ab.cross(*abc);

    // origin outside AB, remove C and v = edge normal towards origin
    if !ignore_ab && ab_perp.dot(*ao) > S::zero() {
        simplex.remove(0);
        *v = cross_aba(ab, ao);
        return;
    }

    let ac_perp = abc.cross(*ac);

    // origin outside AC, remove B and v = edge normal towards origin
    if ac_perp.dot(*ao) > S::zero() {
        simplex.remove(1);
        *v = cross_aba(ac, ao);
        return;
        // origin above triangle, set v = surface normal towards origin
    }

    if above {
        *v = *abc;
    } else if abc.dot(*ao) > S::zero() {
        // [c, b, a]
        *v = *abc;
    // origin below triangle, rewind simplex and set v = surface normal towards origin
    } else {
        // [b, c, a]
        simplex.swap(0, 1);
        *v = abc.neg();
    }
}

#[cfg(test)]
mod tests {
    use std::ops::Neg;

    use cgmath::{Point3, Vector3};

    use super::*;
    use algorithm::minkowski::SupportPoint;

    #[test]
    fn test_check_side_outside_ab() {
        let mut simplex = vec![sup(8., -10., 0.), sup(-1., -10., 0.), sup(3., 5., 0.)];
        let v = test_check_side(&mut simplex, false, false);
        assert_eq!(2, simplex.len());
        assert_eq!(Vector3::new(-1., -10., 0.), simplex[0].v); // B should be last in the simplex
        assert_ulps_eq!(-375., v.x);
        assert_ulps_eq!(100., v.y);
        assert_ulps_eq!(0., v.z);
    }

    #[test]
    fn test_check_side_outside_ac() {
        let mut simplex = vec![sup(2., -10., 0.), sup(-7., -10., 0.), sup(-3., 5., 0.)];
        let v = test_check_side(&mut simplex, false, false);
        assert_eq!(2, simplex.len());
        assert_eq!(Vector3::new(2., -10., 0.), simplex[0].v); // C should be last in the simplex
        assert_ulps_eq!(300., v.x);
        assert_ulps_eq!(100., v.y);
        assert_ulps_eq!(0., v.z);
    }

    #[test]
    fn test_check_side_above() {
        let mut simplex = vec![sup(5., -10., -1.), sup(-4., -10., -1.), sup(0., 5., -1.)];
        let v = test_check_side(&mut simplex, false, false);
        assert_eq!(3, simplex.len());
        assert_eq!(Vector3::new(5., -10., -1.), simplex[0].v); // C should be last in the simplex
        assert_ulps_eq!(0., v.x);
        assert_ulps_eq!(0., v.y);
        assert_ulps_eq!(135., v.z);
    }

    #[test]
    fn test_check_side_below() {
        let mut simplex = vec![sup(5., -10., 1.), sup(-4., -10., 1.), sup(0., 5., 1.)];
        let v = test_check_side(&mut simplex, false, false);
        assert_eq!(3, simplex.len());
        assert_eq!(Vector3::new(-4., -10., 1.), simplex[0].v); // B should be last in the simplex
        assert_ulps_eq!(0., v.x);
        assert_ulps_eq!(0., v.y);
        assert_ulps_eq!(-135., v.z);
    }

    #[test]
    fn test_check_origin_empty() {
        let mut simplex = vec![];
        let (hit, v) = test_check_origin(&mut simplex);
        assert!(!hit);
        assert!(simplex.is_empty());
        assert_eq!(Vector3::zero(), v);
    }

    #[test]
    fn test_check_origin_point() {
        let mut simplex = vec![sup(8., -10., 0.)];
        let (hit, v) = test_check_origin(&mut simplex);
        assert!(!hit);
        assert_eq!(1, simplex.len());
        assert_eq!(Vector3::zero(), v);
    }

    #[test]
    fn test_check_origin_line() {
        let mut simplex = vec![sup(8., -10., 0.), sup(-1., -10., 0.)];
        let (hit, v) = test_check_origin(&mut simplex);
        assert!(!hit);
        assert_eq!(2, simplex.len());
        assert_eq!(Vector3::new(0., 810., 0.), v);
    }

    #[test]
    fn test_check_origin_triangle() {
        let mut simplex = vec![sup(5., -10., -1.), sup(-4., -10., -1.), sup(0., 5., -1.)];
        let (hit, v) = test_check_origin(&mut simplex);
        assert!(!hit);
        assert_eq!(3, simplex.len());
        assert_eq!(Vector3::new(0., 0., 135.), v);
    }

    #[test]
    fn test_check_origin_tetrahedron_outside_abc() {
        let mut simplex = vec![
            sup(8., -10., -1.),
            sup(-1., -10., -1.),
            sup(3., 5., -1.),
            sup(3., -3., 5.),
        ];
        let (hit, v) = test_check_origin(&mut simplex);
        assert!(!hit);
        assert_eq!(3, simplex.len());
        assert_eq!(Vector3::new(-1., -10., -1.), simplex[0].v);
        assert_eq!(Vector3::new(-90., 24., 32.), v);
    }

    #[test]
    fn test_check_origin_tetrahedron_outside_acd() {
        let mut simplex = vec![
            sup(8., 0.1, -1.),
            sup(-1., 0.1, -1.),
            sup(3., 15., -1.),
            sup(3., 7., 5.),
        ];
        let (hit, v) = test_check_origin(&mut simplex);
        assert!(!hit);
        assert_eq!(3, simplex.len());
        assert_eq!(Vector3::new(3., 7., 5.), simplex[2].v);
        assert_eq!(Vector3::new(-1., 0.1, -1.), simplex[1].v);
        assert_eq!(Vector3::new(8., 0.1, -1.), simplex[0].v);
        assert_eq!(Vector3::new(0., -54., 62.1), v);
    }

    #[test]
    fn test_check_origin_tetrahedron_outside_adb() {
        let mut simplex = vec![
            sup(2., -10., -1.),
            sup(-7., -10., -1.),
            sup(-3., 5., -1.),
            sup(-3., -3., 5.),
        ];
        let (hit, v) = test_check_origin(&mut simplex);
        assert!(!hit);
        assert_eq!(3, simplex.len());
        assert_eq!(Vector3::new(-3., -3., 5.), simplex[2].v);
        assert_eq!(Vector3::new(2., -10., -1.), simplex[1].v);
        assert_eq!(Vector3::new(-3., 5., -1.), simplex[0].v);
        assert_eq!(Vector3::new(90., 30., 40.), v);
    }

    #[test]
    fn test_check_origin_tetrahedron_inside() {
        let mut simplex = vec![
            sup(3., -3., -1.),
            sup(-3., -3., -1.),
            sup(0., 3., -1.),
            sup(0., 0., 5.),
        ];
        let (hit, _) = test_check_origin(&mut simplex);
        assert!(hit);
        assert_eq!(4, simplex.len());
    }

    fn test_check_origin(simplex: &mut Vec<SupportPoint<Point3<f32>>>) -> (bool, Vector3<f32>) {
        let mut v = Vector3::zero();
        let b = SimplexProcessor3::new().check_origin(simplex, &mut v);
        (b, v)
    }

    fn test_check_side(
        simplex: &mut Vec<SupportPoint<Point3<f32>>>,
        above: bool,
        ignore: bool,
    ) -> Vector3<f32> {
        let ab = simplex[1].v - simplex[2].v;
        let ac = simplex[0].v - simplex[2].v;
        let ao = simplex[2].v.neg();
        let abc = ab.cross(ac);
        let mut v = Vector3::zero();
        check_side(&abc, &ab, &ac, &ao, simplex, &mut v, above, ignore);
        v
    }

    fn sup(x: f32, y: f32, z: f32) -> SupportPoint<Point3<f32>> {
        let mut s = SupportPoint::new();
        s.v = Vector3::new(x, y, z);
        s
    }
}
