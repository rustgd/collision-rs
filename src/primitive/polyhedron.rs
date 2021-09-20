use std::cmp::Ordering;
use std::collections::HashMap;

use bit_set::BitSet;
use cgmath::prelude::*;
use cgmath::{BaseFloat, Point3, Vector3};

use crate::prelude::*;
use crate::primitive::util::barycentric_point;
use crate::volume::Sphere;
use crate::{Aabb3, Plane, Ray3};

#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(
    feature = "serde",
    derive(Serialize, Deserialize),
    serde(crate = "serde_crate")
)]
enum PolyhedronMode {
    VertexOnly,
    HalfEdge,
}

#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(
    feature = "serde",
    derive(Serialize, Deserialize),
    serde(crate = "serde_crate")
)]
struct Vertex<S> {
    position: Point3<S>,
    edge: usize,
    ready: bool,
}

#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(
    feature = "serde",
    derive(Serialize, Deserialize),
    serde(crate = "serde_crate")
)]
struct Edge {
    target_vertex: usize,
    left_face: usize,
    next_edge: usize,
    previous_edge: usize,
    twin_edge: usize,
    ready: bool,
}

#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(
    feature = "serde",
    derive(Serialize, Deserialize),
    serde(crate = "serde_crate")
)]
struct Face<S>
where
    S: BaseFloat,
{
    edge: usize,
    vertices: (usize, usize, usize),
    plane: Plane<S>,
    ready: bool,
}

/// Convex polyhedron primitive.
///
/// Can contain any number of vertices, but a high number of vertices will
/// affect performance of course. It is recommended for high vertex counts, to also provide the
/// faces, this will cause the support function to use hill climbing on a half edge structure,
/// resulting in better performance. The breakpoint is around 250 vertices, but the face version is
/// only marginally slower on lower vertex counts (about 1-2%), while for higher vertex counts it's
/// about 2-5 times faster.
///
///
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(
    feature = "serde",
    derive(Serialize, Deserialize),
    serde(crate = "serde_crate")
)]
pub struct ConvexPolyhedron<S>
where
    S: BaseFloat,
{
    mode: PolyhedronMode,
    vertices: Vec<Vertex<S>>,
    edges: Vec<Edge>,
    faces: Vec<Face<S>>,
    bound: Aabb3<S>,
    max_extent: S,
}

impl<S> ConvexPolyhedron<S>
where
    S: BaseFloat,
{
    /// Create a new convex polyhedron from the given vertices.
    pub fn new(vertices: &[Point3<S>]) -> Self {
        Self {
            mode: PolyhedronMode::VertexOnly,
            vertices: vertices
                .iter()
                .map(|&v| Vertex {
                    position: v,
                    edge: 0,
                    ready: true,
                })
                .collect(),
            edges: Vec::default(),
            faces: Vec::default(),
            bound: vertices
                .iter()
                .fold(Aabb3::zero(), |bound, p| bound.grow(*p)),
            max_extent: vertices
                .iter()
                .map(|p| p.to_vec().magnitude())
                .max_by(|a, b| a.partial_cmp(b).unwrap_or(Ordering::Equal))
                .unwrap_or_else(S::zero),
        }
    }

    /// Create a new convex polyhedron from the given vertices and faces.
    pub fn new_with_faces(vertices: &[Point3<S>], faces: &[(usize, usize, usize)]) -> Self {
        let (vertices, edges, faces) = build_half_edges(vertices, faces);
        Self {
            mode: PolyhedronMode::HalfEdge,
            bound: vertices
                .iter()
                .fold(Aabb3::zero(), |bound, p| bound.grow(p.position)),
            max_extent: vertices
                .iter()
                .map(|p| p.position.to_vec().magnitude())
                .max_by(|a, b| a.partial_cmp(b).unwrap_or(Ordering::Equal))
                .unwrap_or_else(S::zero),
            vertices,
            edges,
            faces,
        }
    }

    /// Create a new convex polyhedron from the given vertices and faces. Will remove any duplicate
    /// vertices.
    pub fn new_with_faces_dedup(vertices: &[Point3<S>], faces: &[(usize, usize, usize)]) -> Self {
        let (vertices, map) = dedup_vertices(vertices);
        Self::new_with_faces(&vertices, &dedup_faces(faces, &map))
    }

    /// Return an iterator that will yield tuples of the 3 vertices of each face
    pub fn faces_iter(&self) -> FaceIterator<'_, S> {
        assert_eq!(self.mode, PolyhedronMode::HalfEdge);
        FaceIterator {
            polyhedron: self,
            current: 0,
        }
    }

    #[inline]
    fn brute_force_support_point(&self, direction: Vector3<S>) -> Point3<S> {
        let (p, _) = self
            .vertices
            .iter()
            .map(|v| (v.position, v.position.dot(direction)))
            .fold(
                (Point3::origin(), S::neg_infinity()),
                |(max_p, max_dot), (v, dot)| {
                    if dot > max_dot {
                        (v, dot)
                    } else {
                        (max_p, max_dot)
                    }
                },
            );
        p
    }

    #[inline]
    fn hill_climb_support_point(&self, direction: Vector3<S>) -> Point3<S> {
        let mut best_index = 0;
        let mut best_dot = self.vertices[best_index].position.dot(direction);

        loop {
            let previous_best_dot = best_dot;
            let mut edge_index = self.vertices[best_index].edge;
            let start_edge_index = edge_index;

            loop {
                let vertex_index = self.edges[edge_index].target_vertex;

                let dot = self.vertices[vertex_index].position.dot(direction);
                if dot > best_dot {
                    best_index = vertex_index;
                    best_dot = dot;
                }

                edge_index = self.edges[self.edges[edge_index].twin_edge].next_edge;
                if start_edge_index == edge_index {
                    break;
                }
            }

            if best_dot == previous_best_dot {
                break;
            }
        }

        self.vertices[best_index].position
    }
}

/// Iterate over polyhedron faces.
/// Yields a tuple with the positions of the 3 vertices of each face
#[derive(Debug)]
pub struct FaceIterator<'a, S>
where
    S: BaseFloat,
{
    polyhedron: &'a ConvexPolyhedron<S>,
    current: usize,
}

impl<'a, S> Iterator for FaceIterator<'a, S>
where
    S: BaseFloat,
{
    type Item = (&'a Point3<S>, &'a Point3<S>, &'a Point3<S>);

    fn next(&mut self) -> Option<Self::Item> {
        if self.current >= self.polyhedron.faces.len() {
            None
        } else {
            self.current += 1;
            let face = &self.polyhedron.faces[self.current - 1];
            Some((
                &self.polyhedron.vertices[face.vertices.0].position,
                &self.polyhedron.vertices[face.vertices.1].position,
                &self.polyhedron.vertices[face.vertices.2].position,
            ))
        }
    }
}

fn dedup_vertices<S>(vertices: &[Point3<S>]) -> (Vec<Point3<S>>, HashMap<usize, usize>)
where
    S: BaseFloat,
{
    let mut vs = Vec::with_capacity(vertices.len() / 2);
    let mut dup = HashMap::default();
    for (i, &vertex) in vertices.iter().enumerate() {
        let mut found = false;
        for (j, &v) in vs.iter().enumerate() {
            if v == vertex {
                dup.insert(i, j);
                found = true;
            }
        }
        if !found {
            vs.push(vertex);
        }
    }
    (vs, dup)
}

fn dedup_faces(
    faces: &[(usize, usize, usize)],
    duplicates: &HashMap<usize, usize>,
) -> Vec<(usize, usize, usize)> {
    faces
        .iter()
        .map(|&(a, b, c)| {
            (
                *duplicates.get(&a).unwrap_or(&a),
                *duplicates.get(&b).unwrap_or(&b),
                *duplicates.get(&c).unwrap_or(&c),
            )
        })
        .collect()
}

/// Create half edge data structure from vertices and faces
fn build_half_edges<S>(
    vertices: &[Point3<S>],
    in_faces: &[(usize, usize, usize)],
) -> (Vec<Vertex<S>>, Vec<Edge>, Vec<Face<S>>)
where
    S: BaseFloat,
{
    let mut vertices: Vec<Vertex<S>> = vertices
        .iter()
        .map(|&v| Vertex {
            position: v,
            edge: 0,
            ready: false,
        })
        .collect();
    let mut edges: Vec<Edge> = vec![];
    let mut faces: Vec<Face<S>> = vec![];
    let mut edge_map: HashMap<(usize, usize), usize> = HashMap::default();
    for &(a, b, c) in in_faces {
        let face_vertices = [a, b, c];
        let mut face = Face {
            edge: 0,
            vertices: (a, b, c),
            plane: Plane::from_points(
                vertices[a].position,
                vertices[b].position,
                vertices[c].position,
            )
            .unwrap(),
            ready: false,
        };
        let face_index = faces.len();
        let mut face_edge_indices = [0, 0, 0];
        for j in 0..3 {
            let i = if j == 0 { 2 } else { j - 1 };
            let v0 = face_vertices[i];
            let v1 = face_vertices[j];

            let (edge_v0_v1_index, edge_v1_v0_index) = if let Some(edge) = edge_map.get(&(v0, v1)) {
                (*edge, edges[*edge].twin_edge)
            } else {
                let edge_v0_v1_index = edges.len();
                let edge_v1_v0_index = edges.len() + 1;

                edges.push(Edge {
                    target_vertex: v1,
                    left_face: 0,
                    next_edge: 0,
                    previous_edge: 0,
                    twin_edge: edge_v1_v0_index,
                    ready: false,
                });

                edges.push(Edge {
                    target_vertex: v0,
                    left_face: 0,
                    next_edge: 0,
                    previous_edge: 0,
                    twin_edge: edge_v0_v1_index,
                    ready: false,
                });

                (edge_v0_v1_index, edge_v1_v0_index)
            };

            edge_map.insert((v0, v1), edge_v0_v1_index);
            edge_map.insert((v1, v0), edge_v1_v0_index);

            if !edges[edge_v0_v1_index].ready {
                edges[edge_v0_v1_index].left_face = face_index;
                edges[edge_v0_v1_index].ready = true;
            }

            if !vertices[v0].ready {
                vertices[v0].edge = edge_v0_v1_index;
                vertices[v0].ready = true;
            }

            if !face.ready {
                face.edge = edge_v0_v1_index;
                face.ready = true;
            }

            face_edge_indices[i] = edge_v0_v1_index;
        }

        faces.push(face);

        for j in 0..3 {
            let i = if j == 0 { 2 } else { j - 1 };
            let edge_i = face_edge_indices[i];
            let edge_j = face_edge_indices[j];
            edges[edge_i].next_edge = edge_j;
            edges[edge_j].previous_edge = edge_i;
        }
    }

    (vertices, edges, faces)
}

impl<S> Primitive for ConvexPolyhedron<S>
where
    S: BaseFloat,
{
    type Point = Point3<S>;

    fn support_point<T>(&self, direction: &Vector3<S>, transform: &T) -> Point3<S>
    where
        T: Transform<Point3<S>>,
    {
        let p = match self.mode {
            PolyhedronMode::VertexOnly => self
                .brute_force_support_point(transform.inverse_transform_vector(*direction).unwrap()),

            PolyhedronMode::HalfEdge => self
                .hill_climb_support_point(transform.inverse_transform_vector(*direction).unwrap()),
        };
        transform.transform_point(p)
    }
}

impl<S> ComputeBound<Aabb3<S>> for ConvexPolyhedron<S>
where
    S: BaseFloat,
{
    fn compute_bound(&self) -> Aabb3<S> {
        self.bound
    }
}

impl<S> ComputeBound<Sphere<S>> for ConvexPolyhedron<S>
where
    S: BaseFloat,
{
    fn compute_bound(&self) -> Sphere<S> {
        Sphere {
            center: Point3::origin(),
            radius: self.max_extent,
        }
    }
}

/// TODO: better algorithm for finding faces to intersect with?
impl<S> Discrete<Ray3<S>> for ConvexPolyhedron<S>
where
    S: BaseFloat,
{
    /// Ray must be in object space
    fn intersects(&self, ray: &Ray3<S>) -> bool {
        find_intersecting_face(self, ray).is_some()
    }
}

impl<S> Continuous<Ray3<S>> for ConvexPolyhedron<S>
where
    S: BaseFloat,
{
    type Result = Point3<S>;

    /// Ray must be in object space
    fn intersection(&self, ray: &Ray3<S>) -> Option<Point3<S>> {
        find_intersecting_face(self, ray).map(|(face_index, (u, v, w))| {
            let f = &self.faces[face_index];
            let v0 = f.vertices.0;
            let v1 = f.vertices.1;
            let v2 = f.vertices.2;
            (self.vertices[v0].position * u)
                + (self.vertices[v1].position.to_vec() * v)
                + (self.vertices[v2].position.to_vec() * w)
        })
    }
}

// TODO: better algorithm for finding faces to intersect with?
// Current algorithm walks in the direction of the plane/ray intersection point for the current
// tested face, assuming the intersection point isn't on the actual face.
// It uses barycentric coordinates to find out where to walk next.
#[inline]
fn find_intersecting_face<S>(
    polytope: &ConvexPolyhedron<S>,
    ray: &Ray3<S>,
) -> Option<(usize, (S, S, S))>
where
    S: BaseFloat,
{
    let mut face = Some(0);
    let mut checked = BitSet::with_capacity(polytope.faces.len());
    while face.is_some() {
        let face_index = face.unwrap();
        checked.insert(face_index);
        let uvw = match intersect_ray_face(ray, polytope, &polytope.faces[face_index]) {
            Some((u, v, w)) => {
                if in_range(u) && in_range(v) && in_range(w) {
                    return Some((face_index, (u, v, w)));
                }
                Some((u, v, w))
            }
            _ => None,
        };
        face = next_face_classify(polytope, face_index, uvw, &mut checked);
    }
    None
}

#[inline]
fn in_range<S>(v: S) -> bool
where
    S: BaseFloat,
{
    v >= S::zero() && v <= S::one()
}

#[inline]
fn next_face_classify<S>(
    polytope: &ConvexPolyhedron<S>,
    face_index: usize,
    bary_coords: Option<(S, S, S)>,
    checked: &mut BitSet,
) -> Option<usize>
where
    S: BaseFloat,
{
    if polytope.faces.len() < 10 {
        if face_index == polytope.faces.len() - 1 {
            None
        } else {
            Some(face_index + 1)
        }
    } else {
        match bary_coords {
            None => {
                let mut next = face_index + 1;
                while next < polytope.faces.len() && checked.contains(next) {
                    next += 1;
                }
                if next == polytope.faces.len() {
                    None
                } else {
                    Some(next)
                }
            }

            Some((u, v, _)) => {
                let face = &polytope.faces[face_index];
                let target_vertex_index = if u < S::zero() {
                    face.vertices.2
                } else if v < S::zero() {
                    face.vertices.0
                } else {
                    face.vertices.1
                };

                let face_edge = &polytope.edges[face.edge];

                let edges = if face_edge.target_vertex == target_vertex_index {
                    [face.edge, face_edge.next_edge, face_edge.previous_edge]
                } else if polytope.edges[face_edge.previous_edge].target_vertex
                    == target_vertex_index
                {
                    [face_edge.previous_edge, face.edge, face_edge.next_edge]
                } else {
                    [face_edge.next_edge, face_edge.previous_edge, face.edge]
                };

                for edge_index in &edges {
                    let twin_edge = polytope.edges[*edge_index].twin_edge;
                    if !checked.contains(polytope.edges[twin_edge].left_face) {
                        return Some(polytope.edges[twin_edge].left_face);
                    }
                }

                for i in 0..polytope.faces.len() {
                    if !checked.contains(i) {
                        return Some(i);
                    }
                }

                None
            }
        }
    }
}

/// Compute intersection point of ray and face in barycentric coordinates.
#[inline]
fn intersect_ray_face<S>(
    ray: &Ray3<S>,
    polytope: &ConvexPolyhedron<S>,
    face: &Face<S>,
) -> Option<(S, S, S)>
where
    S: BaseFloat,
{
    let n_dir = face.plane.n.dot(ray.direction);
    if n_dir < S::zero() {
        let v0 = face.vertices.0;
        let v1 = face.vertices.1;
        let v2 = face.vertices.2;
        face.plane.intersection(ray).map(|p| {
            barycentric_point(
                p,
                polytope.vertices[v0].position,
                polytope.vertices[v1].position,
                polytope.vertices[v2].position,
            )
        })
    } else {
        None
    }
}

#[cfg(test)]
mod tests {

    use cgmath::assert_ulps_eq;
    use cgmath::prelude::*;
    use cgmath::{Decomposed, Point3, Quaternion, Rad, Vector3};

    use super::ConvexPolyhedron;
    use crate::prelude::*;
    use crate::{Aabb3, Ray3};

    #[test]
    fn test_polytope_half_edge() {
        let vertices = vec![
            Point3::<f32>::new(1., 0., 0.),
            Point3::<f32>::new(0., 1., 0.),
            Point3::<f32>::new(0., 0., 1.),
            Point3::<f32>::new(0., 0., 0.),
        ];
        let faces = vec![(1, 3, 2), (3, 1, 0), (2, 0, 1), (0, 2, 3)];

        let polytope_with_faces = ConvexPolyhedron::new_with_faces(&vertices, &faces);
        let polytope = ConvexPolyhedron::new(&vertices);

        let t = transform(0., 0., 0., 0.);

        let direction = Vector3::new(1., 0., 0.);
        assert_eq!(
            Point3::new(1., 0., 0.),
            polytope.support_point(&direction, &t)
        );
        assert_eq!(
            Point3::new(1., 0., 0.),
            polytope_with_faces.support_point(&direction, &t)
        );

        let direction = Vector3::new(0., 1., 0.);
        assert_eq!(
            Point3::new(0., 1., 0.),
            polytope.support_point(&direction, &t)
        );
        assert_eq!(
            Point3::new(0., 1., 0.),
            polytope_with_faces.support_point(&direction, &t)
        );
    }

    #[test]
    fn test_polytope_bound() {
        let vertices = vec![
            Point3::<f32>::new(1., 0., 0.),
            Point3::<f32>::new(0., 1., 0.),
            Point3::<f32>::new(0., 0., 1.),
            Point3::<f32>::new(0., 0., 0.),
        ];
        let faces = vec![(1, 3, 2), (3, 1, 0), (2, 0, 1), (0, 2, 3)];

        let polytope = ConvexPolyhedron::new_with_faces(&vertices, &faces);
        assert_eq!(
            Aabb3::new(Point3::new(0., 0., 0.), Point3::new(1., 1., 1.)),
            polytope.compute_bound()
        );
    }

    #[test]
    fn test_ray_discrete() {
        let vertices = vec![
            Point3::<f32>::new(1., 0., 0.),
            Point3::<f32>::new(0., 1., 0.),
            Point3::<f32>::new(0., 0., 1.),
            Point3::<f32>::new(0., 0., 0.),
        ];
        let faces = vec![(1, 3, 2), (3, 1, 0), (2, 0, 1), (0, 2, 3)];

        let polytope = ConvexPolyhedron::new_with_faces(&vertices, &faces);
        let ray = Ray3::new(Point3::new(0.25, 5., 0.25), Vector3::new(0., -1., 0.));
        assert!(polytope.intersects(&ray));
        let ray = Ray3::new(Point3::new(0.5, 5., 0.5), Vector3::new(0., 1., 0.));
        assert!(!polytope.intersects(&ray));
    }

    #[test]
    fn test_ray_discrete_transformed() {
        let vertices = vec![
            Point3::<f32>::new(1., 0., 0.),
            Point3::<f32>::new(0., 1., 0.),
            Point3::<f32>::new(0., 0., 1.),
            Point3::<f32>::new(0., 0., 0.),
        ];
        let faces = vec![(1, 3, 2), (3, 1, 0), (2, 0, 1), (0, 2, 3)];
        let polytope = ConvexPolyhedron::new_with_faces(&vertices, &faces);
        let t = transform(0., 0., 0., 0.);
        let ray = Ray3::new(Point3::new(0.25, 5., 0.25), Vector3::new(0., -1., 0.));
        assert!(polytope.intersects_transformed(&ray, &t));
        let ray = Ray3::new(Point3::new(0.5, 5., 0.5), Vector3::new(0., 1., 0.));
        assert!(!polytope.intersects_transformed(&ray, &t));
        let t = transform(0., 1., 0., 0.);
        let ray = Ray3::new(Point3::new(0.25, 5., 0.25), Vector3::new(0., -1., 0.));
        assert!(polytope.intersects_transformed(&ray, &t));
        let t = transform(0., 0., 0., 0.3);
        assert!(polytope.intersects_transformed(&ray, &t));
    }

    #[test]
    fn test_ray_continuous() {
        let vertices = vec![
            Point3::<f32>::new(1., 0., 0.),
            Point3::<f32>::new(0., 1., 0.),
            Point3::<f32>::new(0., 0., 1.),
            Point3::<f32>::new(0., 0., 0.),
        ];
        let faces = vec![(1, 3, 2), (3, 1, 0), (2, 0, 1), (0, 2, 3)];

        let polytope = ConvexPolyhedron::new_with_faces(&vertices, &faces);
        let ray = Ray3::new(Point3::new(0.25, 5., 0.25), Vector3::new(0., -1., 0.));
        let p = polytope.intersection(&ray).unwrap();
        assert_ulps_eq!(0.250_000_18, p.x);
        assert_ulps_eq!(0.499_999_7, p.y);
        assert_ulps_eq!(0.250_000_18, p.z);
        let ray = Ray3::new(Point3::new(0.5, 5., 0.5), Vector3::new(0., 1., 0.));
        assert_eq!(None, polytope.intersection(&ray));
        let ray = Ray3::new(Point3::new(0., 5., 0.), Vector3::new(0., -1., 0.));
        assert_eq!(Some(Point3::new(0., 1., 0.)), polytope.intersection(&ray));
    }

    #[test]
    fn test_ray_continuous_transformed() {
        let vertices = vec![
            Point3::<f32>::new(1., 0., 0.),
            Point3::<f32>::new(0., 1., 0.),
            Point3::<f32>::new(0., 0., 1.),
            Point3::<f32>::new(0., 0., 0.),
        ];
        let faces = vec![(1, 3, 2), (3, 1, 0), (2, 0, 1), (0, 2, 3)];
        let polytope = ConvexPolyhedron::new_with_faces(&vertices, &faces);
        let t = transform(0., 0., 0., 0.);
        let ray = Ray3::new(Point3::new(0.25, 5., 0.25), Vector3::new(0., -1., 0.));
        let p = polytope.intersection_transformed(&ray, &t).unwrap();
        assert_ulps_eq!(0.250_000_18, p.x);
        assert_ulps_eq!(0.499_999_7, p.y);
        assert_ulps_eq!(0.250_000_18, p.z);
        let ray = Ray3::new(Point3::new(0.5, 5., 0.5), Vector3::new(0., 1., 0.));
        assert_eq!(None, polytope.intersection_transformed(&ray, &t));
        let t = transform(0., 1., 0., 0.);
        let ray = Ray3::new(Point3::new(0.25, 5., 0.25), Vector3::new(0., -1., 0.));
        let p = polytope.intersection_transformed(&ray, &t).unwrap();
        assert_ulps_eq!(0.250_000_18, p.x);
        assert_ulps_eq!(1.499_999_7, p.y);
        assert_ulps_eq!(0.250_000_18, p.z);
        let t = transform(0., 0., 0., 0.3);
        let p = polytope.intersection_transformed(&ray, &t).unwrap();
        assert_ulps_eq!(0.25, p.x);
        assert_ulps_eq!(0.467_716_2, p.y);
        assert_ulps_eq!(0.25, p.z);
    }

    #[test]
    fn test_intersect_face() {
        let vertices = vec![
            Point3::<f32>::new(1., 0., 0.),
            Point3::<f32>::new(0., 1., 0.),
            Point3::<f32>::new(0., 0., 1.),
            Point3::<f32>::new(0., 0., 0.),
        ];
        let faces = vec![(1, 3, 2), (3, 1, 0), (2, 0, 1), (0, 2, 3)];
        let polytope = ConvexPolyhedron::new_with_faces(&vertices, &faces);
        let ray = Ray3::new(Point3::new(1., -1., 1.), Vector3::new(0., 1., 0.));
        polytope.intersection(&ray);
    }

    fn transform(dx: f32, dy: f32, dz: f32, rot: f32) -> Decomposed<Vector3<f32>, Quaternion<f32>> {
        Decomposed {
            scale: 1.,
            rot: Quaternion::from_angle_z(Rad(rot)),
            disp: Vector3::new(dx, dy, dz),
        }
    }
}
