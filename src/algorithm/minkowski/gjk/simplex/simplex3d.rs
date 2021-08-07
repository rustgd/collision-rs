use std::marker;
use std::ops::Neg;

use cgmath::num_traits::cast;
use cgmath::prelude::*;
use cgmath::ulps_eq;
use cgmath::{BaseFloat, Point3, Vector3};

use super::{Simplex, SimplexProcessor};
use crate::primitive::util::{barycentric_vector, get_closest_point_on_edge};

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

    fn reduce_to_closest_feature(
        &self,
        simplex: &mut Simplex<Point3<S>>,
        v: &mut Vector3<S>,
    ) -> bool {
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
            if ulps_eq!(*v, Vector3::zero()) {
                v.x = cast(0.1).unwrap();
            }
        }
        // 0-1 points
        false
    }

    /// Get the closest point on the simplex to the origin.
    ///
    /// Make simplex only retain the closest feature to the origin.
    fn get_closest_point_to_origin(&self, simplex: &mut Simplex<Point3<S>>) -> Vector3<S> {
        let mut d = Vector3::zero();

        // reduce simplex to the closest feature to the origin
        // if check_origin return true, the origin is inside the simplex, so return the zero vector
        // if not, the simplex will be the closest face or edge to the origin, and d the normal of
        // the feature in the direction of the origin
        if self.reduce_to_closest_feature(simplex, &mut d) {
            return d;
        }

        if simplex.len() == 1 {
            simplex[0].v
        } else if simplex.len() == 2 {
            get_closest_point_on_edge(&simplex[1].v, &simplex[0].v, &Vector3::zero())
        } else {
            get_closest_point_on_face(&simplex[2].v, &simplex[1].v, &simplex[0].v, &d)
        }
    }

    fn new() -> Self {
        Self {
            m: marker::PhantomData,
        }
    }
}

#[inline]
fn get_closest_point_on_face<S>(
    a: &Vector3<S>,
    b: &Vector3<S>,
    c: &Vector3<S>,
    normal: &Vector3<S>,
) -> Vector3<S>
where
    S: BaseFloat,
{
    use crate::{Continuous, Plane, Ray3};
    let ap = Point3::from_vec(*a);
    let bp = Point3::from_vec(*b);
    let cp = Point3::from_vec(*c);
    let ray = Ray3::new(Point3::origin(), -*normal);
    // unwrapping is safe, because the degenerate face will have been removed by the outer algorithm
    let plane = Plane::from_points(ap, bp, cp).unwrap();
    match plane.intersection(&ray) {
        Some(point) => {
            let (u, v, w) = barycentric_vector(point.to_vec(), *a, *b, *c);
            assert!(
                in_range(u) && in_range(v) && in_range(w),
                "Simplex processing failed to deduce that this simplex {:?} is an edge case",
                [a, b, c]
            );

            point.to_vec()
        }

        _ => Vector3::zero(),
    }
}

#[inline]
fn in_range<S>(v: S) -> bool
where
    S: BaseFloat,
{
    v >= S::zero() && v <= S::one()
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
    simplex: &mut Simplex<Point3<S>>,
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

    if above || abc.dot(*ao) > S::zero() {
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

    use cgmath::assert_ulps_eq;
    use cgmath::{Point3, Vector3};
    use smallvec::smallvec;

    use super::*;
    use crate::algorithm::minkowski::SupportPoint;

    #[test]
    fn test_check_side_outside_ab() {
        let mut simplex = smallvec![sup(8., -10., 0.), sup(-1., -10., 0.), sup(3., 5., 0.)];
        let v = test_check_side(&mut simplex, false, false);
        assert_eq!(2, simplex.len());
        assert_eq!(Vector3::new(-1., -10., 0.), simplex[0].v); // B should be last in the simplex
        assert_ulps_eq!(-375., v.x);
        assert_ulps_eq!(100., v.y);
        assert_ulps_eq!(0., v.z);
    }

    #[test]
    fn test_check_side_outside_ac() {
        let mut simplex = smallvec![sup(2., -10., 0.), sup(-7., -10., 0.), sup(-3., 5., 0.)];
        let v = test_check_side(&mut simplex, false, false);
        assert_eq!(2, simplex.len());
        assert_eq!(Vector3::new(2., -10., 0.), simplex[0].v); // C should be last in the simplex
        assert_ulps_eq!(300., v.x);
        assert_ulps_eq!(100., v.y);
        assert_ulps_eq!(0., v.z);
    }

    #[test]
    fn test_check_side_above() {
        let mut simplex = smallvec![sup(5., -10., -1.), sup(-4., -10., -1.), sup(0., 5., -1.)];
        let v = test_check_side(&mut simplex, false, false);
        assert_eq!(3, simplex.len());
        assert_eq!(Vector3::new(5., -10., -1.), simplex[0].v); // C should be last in the simplex
        assert_ulps_eq!(0., v.x);
        assert_ulps_eq!(0., v.y);
        assert_ulps_eq!(135., v.z);
    }

    #[test]
    fn test_check_side_below() {
        let mut simplex = smallvec![sup(5., -10., 1.), sup(-4., -10., 1.), sup(0., 5., 1.)];
        let v = test_check_side(&mut simplex, false, false);
        assert_eq!(3, simplex.len());
        assert_eq!(Vector3::new(-4., -10., 1.), simplex[0].v); // B should be last in the simplex
        assert_ulps_eq!(0., v.x);
        assert_ulps_eq!(0., v.y);
        assert_ulps_eq!(-135., v.z);
    }

    #[test]
    fn test_check_origin_empty() {
        let mut simplex = smallvec![];
        let (hit, v) = test_check_origin(&mut simplex);
        assert!(!hit);
        assert!(simplex.is_empty());
        assert_eq!(Vector3::zero(), v);
    }

    #[test]
    fn test_check_origin_point() {
        let mut simplex = smallvec![sup(8., -10., 0.)];
        let (hit, v) = test_check_origin(&mut simplex);
        assert!(!hit);
        assert_eq!(1, simplex.len());
        assert_eq!(Vector3::zero(), v);
    }

    #[test]
    fn test_check_origin_line() {
        let mut simplex = smallvec![sup(8., -10., 0.), sup(-1., -10., 0.)];
        let (hit, v) = test_check_origin(&mut simplex);
        assert!(!hit);
        assert_eq!(2, simplex.len());
        assert_eq!(Vector3::new(0., 810., 0.), v);
    }

    #[test]
    fn test_check_origin_triangle() {
        let mut simplex = smallvec![sup(5., -10., -1.), sup(-4., -10., -1.), sup(0., 5., -1.)];
        let (hit, v) = test_check_origin(&mut simplex);
        assert!(!hit);
        assert_eq!(3, simplex.len());
        assert_eq!(Vector3::new(0., 0., 135.), v);
    }

    #[test]
    fn test_check_origin_tetrahedron_outside_abc() {
        let mut simplex = smallvec![
            sup(8., -10., -1.),
            sup(-1., -10., -1.),
            sup(3., 5., -1.),
            sup(3., -3., 5.)
        ];
        let (hit, v) = test_check_origin(&mut simplex);
        assert!(!hit);
        assert_eq!(3, simplex.len());
        assert_eq!(Vector3::new(-1., -10., -1.), simplex[0].v);
        assert_eq!(Vector3::new(-90., 24., 32.), v);
    }

    #[test]
    fn test_check_origin_tetrahedron_outside_acd() {
        let mut simplex = smallvec![
            sup(8., 0.1, -1.),
            sup(-1., 0.1, -1.),
            sup(3., 15., -1.),
            sup(3., 7., 5.)
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
        let mut simplex = smallvec![
            sup(2., -10., -1.),
            sup(-7., -10., -1.),
            sup(-3., 5., -1.),
            sup(-3., -3., 5.)
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
        let mut simplex = smallvec![
            sup(3., -3., -1.),
            sup(-3., -3., -1.),
            sup(0., 3., -1.),
            sup(0., 0., 5.)
        ];
        let (hit, _) = test_check_origin(&mut simplex);
        assert!(hit);
        assert_eq!(4, simplex.len());
    }

    fn test_check_origin(simplex: &mut Simplex<Point3<f32>>) -> (bool, Vector3<f32>) {
        let mut v = Vector3::zero();
        let b = SimplexProcessor3::new().reduce_to_closest_feature(simplex, &mut v);
        (b, v)
    }

    fn test_check_side(
        simplex: &mut Simplex<Point3<f32>>,
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
