//! GJK distance/collision detection algorithm. For now only have implementation of collision
//! detection, not distance computation.

pub use self::simplex::SimplexProcessor;

use std::ops::Neg;

use cgmath::BaseFloat;
use cgmath::prelude::*;

use self::simplex::{SimplexProcessor2, SimplexProcessor3};
use {CollisionStrategy, Contact};
use algorithm::minkowski::{EPA2, EPA3, SupportPoint, EPA};
use prelude::*;

mod simplex;

const MAX_ITERATIONS: u32 = 100;

/// GJK algorithm for 2D, see [GJK](struct.GJK.html) for more information.
pub type GJK2<S> = GJK<SimplexProcessor2<S>, EPA2<S>>;

/// GJK algorithm for 3D, see [GJK](struct.GJK.html) for more information.
pub type GJK3<S> = GJK<SimplexProcessor3<S>, EPA3<S>>;

/// Gilbert-Johnson-Keerthi narrow phase collision detection algorithm.
///
/// # Type parameters:
///
/// - `S`: simplex processor type. Should be either
///        [`SimplexProcessor2`](struct.SimplexProcessor2.html) or
///        [`SimplexProcessor3`](struct.SimplexProcessor3.html)
/// - `E`: EPA algorithm implementation type. Should be either
///        [`EPA2`](struct.EPA2.html) or
///        [`EPA3`](struct.EPA3.html)
///
#[derive(Debug)]
pub struct GJK<SP, E> {
    simplex_processor: SP,
    epa: E,
}

impl<SP, E> GJK<SP, E>
where
    SP: SimplexProcessor,
    <SP::Point as EuclideanSpace>::Scalar: BaseFloat,
    E: EPA<Point = SP::Point>,
{
    /// Create a new GJK algorithm implementation
    pub fn new() -> Self {
        Self {
            simplex_processor: SP::new(),
            epa: E::new(),
        }
    }

    /// Do intersection test on the given primitives
    ///
    /// ## Parameters:
    ///
    /// - `left`: left primitive
    /// - `left_transform`: model-to-world-transform for the left primitive
    /// - `right`: right primitive,
    /// - `right_transform`: model-to-world-transform for the right primitive
    ///
    /// ## Returns:
    ///
    /// Will return a simplex if a collision was detected. For 2D, the simplex will be a triangle,
    /// for 3D, it will be a tetrahedron. The simplex will enclose the origin.
    /// If no collision was detected, None is returned.
    ///
    pub fn intersect<P, PL, PR, TL, TR>(
        &self,
        left: &PL,
        left_transform: &TL,
        right: &PR,
        right_transform: &TR,
    ) -> Option<Vec<SupportPoint<P>>>
    where
        P: EuclideanSpace,
        P::Scalar: BaseFloat,
        PL: SupportFunction<Point = P>,
        PR: SupportFunction<Point = P>,
        SP: SimplexProcessor<Point = P>,
        P::Diff: Neg<Output = P::Diff> + InnerSpace,
        TL: Transform<P>,
        TR: Transform<P>,
    {
        let right_pos = right_transform.transform_point(P::origin());
        let left_pos = left_transform.transform_point(P::origin());
        let mut d = right_pos - left_pos;
        let a = SupportPoint::from_minkowski(left, left_transform, right, right_transform, &d);
        if a.v.dot(d) <= P::Scalar::zero() {
            return None;
        }
        let mut simplex: Vec<SupportPoint<P>> = Vec::default();
        simplex.push(a);
        d = d.neg();
        let mut i = 0;
        loop {
            let a = SupportPoint::from_minkowski(left, left_transform, right, right_transform, &d);
            if a.v.dot(d) <= P::Scalar::zero() {
                return None;
            } else {
                simplex.push(a);
                if self.simplex_processor.check_origin(&mut simplex, &mut d) {
                    return Some(simplex);
                }
            }
            i += 1;
            if i >= MAX_ITERATIONS {
                return None;
            }
        }
    }

    /// Given a GJK simplex that encloses the origin, compute the contact manifold.
    ///
    /// Uses the EPA algorithm to find the contact information from the simplex.
    pub fn get_contact_manifold<P, PL, PR, TL, TR>(
        &self,
        mut simplex: &mut Vec<SupportPoint<P>>,
        left: &PL,
        left_transform: &TL,
        right: &PR,
        right_transform: &TR,
    ) -> Option<Contact<P>>
    where
        P: EuclideanSpace,
        PL: SupportFunction<Point = P>,
        PR: SupportFunction<Point = P>,
        TL: Transform<P>,
        TR: Transform<P>,
        SP: SimplexProcessor<Point = P>,
    {
        self.epa
            .process(&mut simplex, left, left_transform, right, right_transform)
    }

    /// Do intersection testing on the given primitives, and return the contact manifold.
    ///
    /// ## Parameters:
    ///
    /// - `strategy`: strategy to use, if `CollisionOnly` it will only return a boolean result,
    ///               otherwise, EPA will be used to compute the exact contact point.
    /// - `left`: left primitive
    /// - `left_transform`: model-to-world-transform for the left primitive
    /// - `right`: right primitive,
    /// - `right_transform`: model-to-world-transform for the right primitive
    ///
    /// ## Returns:
    ///
    /// Will optionally return a `Contact` if a collision was detected. In `CollisionOnly` mode,
    /// this contact will only be a boolean result. For `FullResolution` mode, the contact will
    /// contain a full manifold (collision normal, penetration depth and contact point).
    pub fn intersection<P, PL, PR, TL, TR>(
        &self,
        strategy: &CollisionStrategy,
        left: &PL,
        left_transform: &TL,
        right: &PR,
        right_transform: &TR,
    ) -> Option<Contact<P>>
    where
        P: EuclideanSpace,
        P::Scalar: BaseFloat,
        P::Diff: Neg<Output = P::Diff> + InnerSpace,
        PL: SupportFunction<Point = P>,
        PR: SupportFunction<Point = P>,
        TL: Transform<P>,
        TR: Transform<P>,
        SP: SimplexProcessor<Point = P>,
    {
        match self.intersect(left, left_transform, right, right_transform) {
            None => None,
            Some(mut simplex) => {
                match *strategy {
                    CollisionStrategy::CollisionOnly => {
                        Some(Contact::new(CollisionStrategy::CollisionOnly))
                    }
                    CollisionStrategy::FullResolution => {
                        self.get_contact_manifold(
                            &mut simplex,
                            left,
                            left_transform,
                            right,
                            right_transform,
                        )
                    }
                }
            }
        }
    }

    /// Do intersection test on the given complex shapes, and return the actual intersection point
    ///
    /// ## Parameters:
    ///
    /// - `strategy`: strategy to use, if `CollisionOnly` it will only return a boolean result,
    ///               otherwise, EPA will be used to compute the exact contact point.
    /// - `left`: shape consisting of a slice of primitive + local-to-model-transform for each
    ///           primitive,
    /// - `left_transform`: model-to-world-transform for the left shape
    /// - `right`: shape consisting of a slice of primitive + local-to-model-transform for each
    ///           primitive,
    /// - `right_transform`: model-to-world-transform for the right shape
    ///
    /// ## Returns:
    ///
    /// Will optionally return a `Contact` if a collision was detected. In `CollisionOnly` mode,
    /// this contact will only be a boolean result. For `FullResolution` mode, the contact will
    /// contain a full manifold (collision normal, penetration depth and contact point), for the
    /// contact with the highest penetration depth.
    pub fn intersection_complex<P, PL, PR, TL, TR>(
        &self,
        strategy: &CollisionStrategy,
        left: &[(PL, TL)],
        left_transform: &TL,
        right: &[(PR, TR)],
        right_transform: &TR,
    ) -> Option<Contact<P>>
    where
        P: EuclideanSpace,
        P::Scalar: BaseFloat,
        P::Diff: Neg<Output = P::Diff> + InnerSpace,
        PL: SupportFunction<Point = P>,
        PR: SupportFunction<Point = P>,
        TL: Transform<P>,
        TR: Transform<P>,
        SP: SimplexProcessor<Point = P>,
    {
        let mut contacts = Vec::default();
        for &(ref left_primitive, ref left_local_transform) in left.iter() {
            let left_transform = left_transform.concat(left_local_transform);
            for &(ref right_primitive, ref right_local_transform) in right.iter() {
                let right_transform = right_transform.concat(right_local_transform);
                match self.intersect(
                    left_primitive,
                    &left_transform,
                    right_primitive,
                    &right_transform,
                ) {
                    None => (),
                    Some(mut simplex) => {
                        match *strategy {
                            CollisionStrategy::CollisionOnly => {
                                return Some(Contact::new(CollisionStrategy::CollisionOnly));
                            }
                            CollisionStrategy::FullResolution => {
                                match self.get_contact_manifold(
                                    &mut simplex,
                                    left_primitive,
                                    &left_transform,
                                    right_primitive,
                                    &right_transform,
                                ) {
                                    Some(contact) => contacts.push(contact),
                                    None => (),
                                };
                            }
                        }
                    }
                };
            }
        }

        // CollisionOnly handling will have returned already if there was a contact, so this
        // scenario will only happen when we have a contact in FullResolution mode
        if contacts.len() > 0 {
            // penetration depth defaults to 0., and can't be nan from EPA,
            // so unwrapping is safe
            contacts
                .iter()
                .max_by(|l, r| {
                    l.penetration_depth
                        .partial_cmp(&r.penetration_depth)
                        .unwrap()
                })
                .cloned()
        } else {
            None
        }
    }
}

#[cfg(test)]
mod tests {
    use cgmath::{Basis2, Decomposed, Point2, Point3, Quaternion, Rad, Rotation2, Rotation3,
                 Vector2, Vector3};

    use super::*;
    use primitive::*;

    fn transform(x: f32, y: f32, angle: f32) -> Decomposed<Vector2<f32>, Basis2<f32>> {
        Decomposed {
            disp: Vector2::new(x, y),
            rot: Rotation2::from_angle(Rad(angle)),
            scale: 1.,
        }
    }

    fn transform_3d(
        x: f32,
        y: f32,
        z: f32,
        angle_z: f32,
    ) -> Decomposed<Vector3<f32>, Quaternion<f32>> {
        Decomposed {
            disp: Vector3::new(x, y, z),
            rot: Quaternion::from_angle_z(Rad(angle_z)),
            scale: 1.,
        }
    }

    #[test]
    fn test_gjk_miss() {
        let left = Rectangle::new(10., 10.);
        let left_transform = transform(15., 0., 0.);
        let right = Rectangle::new(10., 10.);
        let right_transform = transform(-15., 0., 0.);
        let gjk = GJK2::new();
        assert!(
            gjk.intersect(&left, &left_transform, &right, &right_transform)
                .is_none()
        );
        assert!(
            gjk.intersection(
                &CollisionStrategy::FullResolution,
                &left,
                &left_transform,
                &right,
                &right_transform
            ).is_none()
        )
    }

    #[test]
    fn test_gjk_hit() {
        let left = Rectangle::new(10., 10.);
        let left_transform = transform(15., 0., 0.);
        let right = Rectangle::new(10., 10.);
        let right_transform = transform(7., 2., 0.);
        let gjk = GJK2::new();
        let simplex = gjk.intersect(&left, &left_transform, &right, &right_transform);
        assert!(simplex.is_some());
        let contact = gjk.intersection(
            &CollisionStrategy::FullResolution,
            &left,
            &left_transform,
            &right,
            &right_transform,
        );
        assert!(contact.is_some());
        let contact = contact.unwrap();
        assert_eq!(Vector2::new(-1., 0.), contact.normal);
        assert_eq!(2., contact.penetration_depth);
        assert_eq!(Point2::new(10., 1.), contact.contact_point);
    }

    #[test]
    fn test_gjk_3d_hit() {
        let left = Cuboid::new(10., 10., 10.);
        let left_transform = transform_3d(15., 0., 0., 0.);
        let right = Cuboid::new(10., 10., 10.);
        let right_transform = transform_3d(7., 2., 0., 0.);
        let gjk = GJK3::new();
        let simplex = gjk.intersect(&left, &left_transform, &right, &right_transform);
        assert!(simplex.is_some());
        let contact = gjk.intersection(
            &CollisionStrategy::FullResolution,
            &left,
            &left_transform,
            &right,
            &right_transform,
        );
        assert!(contact.is_some());
        let contact = contact.unwrap();
        println!("{:?}", contact);
        assert_eq!(Vector3::new(-1., 0., 0.), contact.normal);
        assert_eq!(2., contact.penetration_depth);
        assert_ulps_eq!(Point3::new(10., 1., 5.), contact.contact_point);
    }
}
