//! Utilities
//!

use std::ops::Neg;

use crate::{Aabb, Ray3};
use cgmath::num_traits::Float;
use cgmath::prelude::*;
use cgmath::{BaseFloat, BaseNum, Vector2};

pub(crate) fn get_max_point<'a, P: 'a, T, I>(vertices: I, direction: &P::Diff, transform: &T) -> P
where
    P: EuclideanSpace,
    P::Scalar: BaseFloat,
    T: Transform<P>,
    I: Iterator<Item = &'a P>,
{
    let direction = transform.inverse_transform_vector(*direction).unwrap();
    let (p, _) = vertices.map(|&v| (v, v.dot(direction))).fold(
        (P::origin(), P::Scalar::neg_infinity()),
        |(max_p, max_dot), (v, dot)| {
            if dot > max_dot {
                (v, dot)
            } else {
                (max_p, max_dot)
            }
        },
    );
    transform.transform_point(p)
}

pub(crate) fn get_bound<'a, I, A: 'a>(vertices: I) -> A
where
    A: Aabb,
    I: Iterator<Item = &'a A::Point>,
{
    vertices.fold(A::zero(), |bound, p| bound.grow(*p))
}

#[allow(dead_code)]
#[inline]
pub(crate) fn triple_product<S>(a: &Vector2<S>, b: &Vector2<S>, c: &Vector2<S>) -> Vector2<S>
where
    S: BaseNum,
{
    let ac = a.x * c.x + a.y * c.y;
    let bc = b.x * c.x + b.y * c.y;
    Vector2::new(b.x * ac - a.x * bc, b.y * ac - a.y * bc)
}

/// Compute barycentric coordinates of p in relation to the triangle defined by (a, b, c).
#[allow(dead_code)]
pub(crate) fn barycentric_vector<V>(p: V, a: V, b: V, c: V) -> (V::Scalar, V::Scalar, V::Scalar)
where
    V: VectorSpace + InnerSpace,
    V::Scalar: BaseFloat,
{
    let v0 = b - a;
    let v1 = c - a;
    let v2 = p - a;
    let d00 = v0.dot(v0);
    let d01 = v0.dot(v1);
    let d11 = v1.dot(v1);
    let d20 = v2.dot(v0);
    let d21 = v2.dot(v1);
    let inv_denom = V::Scalar::one() / (d00 * d11 - d01 * d01);

    let v = (d11 * d20 - d01 * d21) * inv_denom;
    let w = (d00 * d21 - d01 * d20) * inv_denom;
    let u = V::Scalar::one() - v - w;
    (u, v, w)
}

/// Compute barycentric coordinates of p in relation to the triangle defined by (a, b, c).
pub(crate) fn barycentric_point<P>(p: P, a: P, b: P, c: P) -> (P::Scalar, P::Scalar, P::Scalar)
where
    P: EuclideanSpace,
    P::Diff: InnerSpace,
    P::Scalar: BaseFloat,
{
    let v0 = b - a;
    let v1 = c - a;
    let v2 = p - a;
    let d00 = v0.dot(v0);
    let d01 = v0.dot(v1);
    let d11 = v1.dot(v1);
    let d20 = v2.dot(v0);
    let d21 = v2.dot(v1);
    let inv_denom = P::Scalar::one() / (d00 * d11 - d01 * d01);

    let v = (d11 * d20 - d01 * d21) * inv_denom;
    let w = (d00 * d21 - d01 * d20) * inv_denom;
    let u = P::Scalar::one() - v - w;
    (u, v, w)
}

#[inline]
pub(crate) fn get_closest_point_on_edge<V>(start: &V, end: &V, point: &V) -> V
where
    V: VectorSpace + InnerSpace + Neg<Output = V>,
    V::Scalar: BaseFloat,
{
    let line = *end - *start;
    let line_dir = line.normalize();
    let v = *point - *start;
    let d = v.dot(line_dir);
    if d < V::Scalar::zero() {
        *start
    } else if (d * d) > line.magnitude2() {
        *end
    } else {
        *start + line_dir * d
    }
}

pub(crate) fn cylinder_ray_quadratic_solve<S>(r: &Ray3<S>, radius: S) -> Option<(S, S)>
where
    S: BaseFloat,
{
    let two = S::one() + S::one();

    let a = r.direction.x * r.direction.x + r.direction.z * r.direction.z;
    let b = two * (r.direction.x * r.origin.x + r.direction.z * r.origin.z);
    let c = r.origin.x * r.origin.x + r.origin.z * r.origin.z - radius * radius;

    let four = two + two;
    let dr = b * b - four * a * c;
    if dr < S::zero() {
        return None;
    }
    let drsqrt = dr.sqrt();
    let t1 = (-b + drsqrt) / (two * a);
    let t2 = (-b - drsqrt) / (two * a);
    Some((t1, t2))
}

#[cfg(test)]
mod tests {
    use std;

    use cgmath::assert_ulps_eq;
    use cgmath::{Basis2, Decomposed, Point2, Rad, Rotation2, Vector2};

    use super::*;
    use crate::Aabb2;

    #[test]
    fn test_get_bound() {
        let triangle = vec![
            Point2::new(-1., 1.),
            Point2::new(0., -1.),
            Point2::new(1., 0.),
        ];
        assert_eq!(
            Aabb2::new(Point2::new(-1., -1.), Point2::new(1., 1.)),
            get_bound(triangle.iter())
        );
    }

    fn test_max_point(dx: f32, dy: f32, px: f32, py: f32, rot_angle: f32) {
        let direction = Vector2::new(dx, dy);
        let point = Point2::new(px, py);
        let triangle = vec![
            Point2::new(-1., 1.),
            Point2::new(0., -1.),
            Point2::new(1., 0.),
        ];
        let t = transform(0., 0., rot_angle);
        let max_point = get_max_point(triangle.iter(), &direction, &t);
        assert_ulps_eq!(point.x, max_point.x);
        assert_ulps_eq!(point.y, max_point.y);
    }

    #[test]
    fn test_max_point_1() {
        test_max_point(0., 1., -1., 1., 0.);
    }

    #[test]
    fn test_max_point_2() {
        test_max_point(-1., 0., -1., 1., 0.);
    }

    #[test]
    fn test_max_point_3() {
        test_max_point(0., -1., 0., -1., 0.);
    }

    #[test]
    fn test_max_point_4() {
        test_max_point(1., 0., 1., 0., 0.);
    }

    #[test]
    fn test_max_point_5() {
        test_max_point(10., 1., 1., 0., 0.);
    }

    #[test]
    fn test_max_point_6() {
        test_max_point(2., -100., 0., -1., 0.);
    }

    #[test]
    fn test_max_point_rot() {
        test_max_point(
            0.,
            1.,
            std::f32::consts::FRAC_1_SQRT_2,
            std::f32::consts::FRAC_1_SQRT_2,
            std::f32::consts::PI / 4.,
        );
    }

    #[test]
    fn test_max_point_disp() {
        let direction = Vector2::new(0., 1.);
        let point = Point2::new(-1., 9.);
        let triangle = vec![
            Point2::new(-1., 1.),
            Point2::new(0., -1.),
            Point2::new(1., 0.),
        ];
        let t = transform(0., 8., 0.);
        assert_eq!(point, get_max_point(triangle.iter(), &direction, &t));
    }

    #[test]
    fn test_closest_point_2d() {
        let start = Vector2::new(3., -1.);
        let end = Vector2::new(-1., 3.);
        let point = Vector2::zero();
        let p = get_closest_point_on_edge(&start, &end, &point);
        assert_ulps_eq!(Vector2::new(1_f32, 1_f32), p);

        let start = Vector2::new(2., -2.);
        let end = Vector2::new(2., 2.);
        let p = get_closest_point_on_edge(&start, &end, &point);
        assert_ulps_eq!(Vector2::new(2_f32, 0_f32), p);

        let start = Vector2::new(2., 4.);
        let end = Vector2::new(2., 2.);
        let p = get_closest_point_on_edge(&start, &end, &point);
        assert_ulps_eq!(Vector2::new(2_f32, 2_f32), p);

        let start = Vector2::new(2., -2.);
        let end = Vector2::new(2., -4.);
        let p = get_closest_point_on_edge(&start, &end, &point);
        assert_ulps_eq!(Vector2::new(2_f32, -2_f32), p);
    }

    fn transform(dx: f32, dy: f32, rot: f32) -> Decomposed<Vector2<f32>, Basis2<f32>> {
        Decomposed {
            scale: 1.,
            rot: Rotation2::from_angle(Rad(rot)),
            disp: Vector2::new(dx, dy),
        }
    }
}
