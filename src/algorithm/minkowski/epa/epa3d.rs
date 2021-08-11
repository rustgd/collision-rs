use std::marker;

use cgmath::num_traits::NumCast;
use cgmath::prelude::*;
use cgmath::{BaseFloat, Point3, Vector3};

use super::SupportPoint;
use super::*;
use crate::prelude::*;
use crate::primitive::util::barycentric_vector;
use crate::{CollisionStrategy, Contact};

/// EPA algorithm implementation for 3D. Only to be used in [`GJK`](struct.GJK.html).
#[derive(Debug)]
pub struct EPA3<S> {
    m: marker::PhantomData<S>,
    tolerance: S,
    max_iterations: u32,
}

impl<S> EPA for EPA3<S>
where
    S: BaseFloat,
{
    type Point = Point3<S>;

    fn process<SL, SR, TL, TR>(
        &self,
        mut simplex: &mut Vec<SupportPoint<Point3<S>>>,
        left: &SL,
        left_transform: &TL,
        right: &SR,
        right_transform: &TR,
    ) -> Option<Contact<Point3<S>>>
    where
        SL: Primitive<Point = Self::Point>,
        SR: Primitive<Point = Self::Point>,
        TL: Transform<Self::Point>,
        TR: Transform<Self::Point>,
    {
        if simplex.len() < 4 {
            return None;
        }
        let mut polytope = Polytope::new(&mut simplex);
        let mut i = 1;
        loop {
            let p = {
                let face = polytope.closest_face_to_origin();
                let p = SupportPoint::from_minkowski(
                    left,
                    left_transform,
                    right,
                    right_transform,
                    &face.normal,
                );
                let d = p.v.dot(face.normal);
                if d - face.distance < self.tolerance || i >= self.max_iterations {
                    return contact(&polytope, face);
                }
                p
            };
            polytope.add(p);
            i += 1;
        }
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

#[inline]
fn contact<S>(polytope: &Polytope<'_, S>, face: &Face<S>) -> Option<Contact<Point3<S>>>
where
    S: BaseFloat,
{
    Some(Contact::new_with_point(
        CollisionStrategy::FullResolution,
        face.normal, // negate ?
        face.distance,
        point(polytope, face),
    ))
}

/// This function returns the contact point in world space coordinates on shape A.
///
/// Compute the closest point to the origin on the given simplex face, then use that to interpolate
/// the support points coming from the A shape.
fn point<S>(polytope: &Polytope<'_, S>, face: &Face<S>) -> Point3<S>
where
    S: BaseFloat,
{
    let (u, v, w) = barycentric_vector(
        face.normal * face.distance,
        polytope.vertices[face.vertices[0]].v,
        polytope.vertices[face.vertices[1]].v,
        polytope.vertices[face.vertices[2]].v,
    );

    polytope.vertices[face.vertices[0]].sup_a * u
        + polytope.vertices[face.vertices[1]].sup_a.to_vec() * v
        + polytope.vertices[face.vertices[2]].sup_a.to_vec() * w
}

#[derive(Debug)]
struct Polytope<'a, S>
where
    S: BaseFloat,
{
    vertices: &'a mut Vec<SupportPoint<Point3<S>>>,
    faces: Vec<Face<S>>,
}

impl<'a, S: 'a> Polytope<'a, S>
where
    S: BaseFloat,
{
    pub fn new(simplex: &'a mut Vec<SupportPoint<Point3<S>>>) -> Self {
        let faces = Face::new(simplex);
        Self {
            vertices: simplex,
            faces,
        }
    }

    pub fn closest_face_to_origin(&'a self) -> &'a Face<S> {
        let mut face = &self.faces[0];
        for f in self.faces[1..].iter() {
            if f.distance < face.distance {
                face = f;
            }
        }
        face
    }

    pub fn add(&mut self, sup: SupportPoint<Point3<S>>) {
        // remove faces that can see the point
        let mut edges = Vec::default();
        let mut i = 0;
        while i < self.faces.len() {
            let dot = self.faces[i]
                .normal
                .dot(sup.v - self.vertices[self.faces[i].vertices[0]].v);
            if dot > S::zero() {
                let face = self.faces.swap_remove(i);
                remove_or_add_edge(&mut edges, (face.vertices[0], face.vertices[1]));
                remove_or_add_edge(&mut edges, (face.vertices[1], face.vertices[2]));
                remove_or_add_edge(&mut edges, (face.vertices[2], face.vertices[0]));
            } else {
                i += 1;
            }
        }

        // add vertex
        let n = self.vertices.len();
        self.vertices.push(sup);

        // add new faces
        let new_faces = edges
            .into_iter()
            .map(|(a, b)| Face::new_impl(self.vertices, n, a, b))
            .collect::<Vec<_>>();
        self.faces.extend(new_faces);
    }
}

#[derive(Debug)]
struct Face<S> {
    pub vertices: [usize; 3],
    pub normal: Vector3<S>,
    pub distance: S,
}

impl<S> Face<S>
where
    S: BaseFloat,
{
    fn new_impl(simplex: &[SupportPoint<Point3<S>>], a: usize, b: usize, c: usize) -> Self {
        let ab = simplex[b].v - simplex[a].v;
        let ac = simplex[c].v - simplex[a].v;
        let normal = ab.cross(ac).normalize();
        let distance = normal.dot(simplex[a].v);
        Self {
            vertices: [a, b, c],
            normal,
            distance,
        }
    }

    pub fn new(simplex: &[SupportPoint<Point3<S>>]) -> Vec<Self> {
        vec![
            Self::new_impl(simplex, 3, 2, 1), // ABC
            Self::new_impl(simplex, 3, 1, 0), // ACD
            Self::new_impl(simplex, 3, 0, 2), // ADB
            Self::new_impl(simplex, 2, 0, 1), // BDC
        ]
    }
}

#[inline]
fn remove_or_add_edge(edges: &mut Vec<(usize, usize)>, edge: (usize, usize)) {
    match edges.iter().position(|e| edge.0 == e.1 && edge.1 == e.0) {
        Some(i) => {
            edges.remove(i);
        }
        None => edges.push(edge),
    }
}

#[cfg(test)]
mod tests {
    use cgmath::assert_ulps_eq;
    use cgmath::{Decomposed, Quaternion, Rad, Vector3};

    use super::*;
    use crate::primitive::*;

    #[test]
    fn test_remove_or_add_edge_added() {
        let mut edges = vec![(1, 2), (6, 5)];
        remove_or_add_edge(&mut edges, (4, 3));
        assert_eq!(3, edges.len());
        assert_eq!((4, 3), edges[2]);
    }

    #[test]
    fn test_remove_or_add_edge_removed() {
        let mut edges = vec![(1, 2), (6, 5)];
        remove_or_add_edge(&mut edges, (2, 1));
        assert_eq!(1, edges.len());
        assert_eq!((6, 5), edges[0]);
    }

    #[test]
    fn test_face_impl() {
        let simplex = vec![
            sup(3., -3., -1.),
            sup(-3., -3., -1.),
            sup(0., 3., -1.),
            sup(0., 0., 5.),
        ];
        let faces = Face::new(&simplex);
        assert_eq!(4, faces.len());
        assert_face(
            &faces[0],
            3,
            2,
            1,
            -0.872_871_5,
            0.436_435_76,
            0.218_217_88,
            1.091_089_4,
        );
        assert_face(
            &faces[1],
            3,
            1,
            0,
            0.,
            -0.894_427_24,
            0.447_213_62,
            2.236_068,
        );
        assert_face(
            &faces[2],
            3,
            0,
            2,
            0.872_871_5,
            0.436_435_76,
            0.218_217_88,
            1.091_089_4,
        );
        assert_face(&faces[3], 2, 0, 1, 0., 0., -1., 1.0);
    }

    #[test]
    fn test_polytope_closest_to_origin() {
        let mut simplex = vec![
            sup(3., -3., -1.),
            sup(-3., -3., -1.),
            sup(0., 3., -1.),
            sup(0., 0., 5.),
        ];
        let polytope = Polytope::new(&mut simplex);
        let face = polytope.closest_face_to_origin();
        assert_face(face, 2, 0, 1, 0., 0., -1., 1.0);
    }

    #[test]
    fn test_polytope_add() {
        let mut simplex = vec![
            sup(3., -3., -1.),
            sup(-3., -3., -1.),
            sup(0., 3., -1.),
            sup(0., 0., 5.),
        ];
        let mut polytope = Polytope::new(&mut simplex);
        polytope.add(sup(0., 0., -2.));
        assert_eq!(5, polytope.vertices.len());
        assert_eq!(6, polytope.faces.len());
        assert_eq!([4, 2, 0], polytope.faces[3].vertices);
        assert_eq!([4, 0, 1], polytope.faces[4].vertices);
        assert_eq!([4, 1, 2], polytope.faces[5].vertices);
    }

    #[test]
    fn test_epa_3d() {
        let left = Cuboid::new(10., 10., 10.);
        let left_transform = transform_3d(15., 0., 0., 0.);
        let right = Cuboid::new(10., 10., 10.);
        let right_transform = transform_3d(7., 2., 0., 0.);
        let mut simplex = vec![
            sup(18., -12., 0.),
            sup(-2., 8., 0.),
            sup(-2., -12., 0.),
            sup(8., -2., -10.),
        ];
        let contact = EPA3::new().process(
            &mut simplex,
            &left,
            &left_transform,
            &right,
            &right_transform,
        );
        assert!(contact.is_some());
        let contact = contact.unwrap();
        assert_eq!(Vector3::new(-1., 0., 0.), contact.normal);
        assert!(2. - contact.penetration_depth <= f32::EPSILON);
    }

    fn assert_face(
        face: &Face<f32>,
        a: usize,
        b: usize,
        c: usize,
        nx: f32,
        ny: f32,
        nz: f32,
        d: f32,
    ) {
        assert_eq!([a, b, c], face.vertices);
        assert_ulps_eq!(nx, face.normal.x);
        assert_ulps_eq!(ny, face.normal.y);
        assert_ulps_eq!(nz, face.normal.z);
        assert_ulps_eq!(d, face.distance);
    }

    fn sup(x: f32, y: f32, z: f32) -> SupportPoint<Point3<f32>> {
        let mut s = SupportPoint::new();
        s.v = Vector3::new(x, y, z);
        s
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
}
