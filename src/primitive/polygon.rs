//! Convex polygon primitive

use cgmath::prelude::*;
use cgmath::{BaseFloat, Point2, Vector2};

use crate::prelude::*;
use crate::primitive::util::{get_bound, get_max_point};
use crate::{Aabb2, Line2, Ray2};

/// Convex polygon primitive.
///
/// Can contain any number of vertices, but a high number of vertices will
/// affect performance of course. Vertices need to be in CCW order.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(
    feature = "serde",
    derive(Serialize, Deserialize),
    serde(crate = "serde_crate")
)]
pub struct ConvexPolygon<S> {
    /// Vertices of the convex polygon.
    pub vertices: Vec<Point2<S>>,
}

impl<S> ConvexPolygon<S> {
    /// Create a new convex polygon from the given vertices. Vertices need to be in CCW order.
    pub fn new(vertices: Vec<Point2<S>>) -> Self {
        Self { vertices }
    }
}

impl<S> Primitive for ConvexPolygon<S>
where
    S: BaseFloat,
{
    type Point = Point2<S>;

    fn support_point<T>(&self, direction: &Vector2<S>, transform: &T) -> Point2<S>
    where
        T: Transform<Point2<S>>,
    {
        if self.vertices.len() < 10 {
            get_max_point(self.vertices.iter(), direction, transform)
        } else {
            support_point(&self.vertices, direction, transform)
        }
    }
}

impl<S> ComputeBound<Aabb2<S>> for ConvexPolygon<S>
where
    S: BaseFloat,
{
    fn compute_bound(&self) -> Aabb2<S> {
        get_bound(self.vertices.iter())
    }
}

impl<S> Discrete<Ray2<S>> for ConvexPolygon<S>
where
    S: BaseFloat,
{
    /// Ray must be in object space
    fn intersects(&self, ray: &Ray2<S>) -> bool {
        for j in 0..self.vertices.len() - 1 {
            let i = if j == 0 {
                self.vertices.len() - 1
            } else {
                j - 1
            };
            let normal = Vector2::new(
                self.vertices[j].y - self.vertices[i].y,
                self.vertices[i].x - self.vertices[j].x,
            );
            // check if edge normal points toward the ray origin
            if ray.direction.dot(normal) < S::zero()
                // check line ray intersection
                && ray.intersection(&Line2::new(self.vertices[i], self.vertices[j]))
                    .is_some()
            {
                return true;
            }
        }

        false
    }
}

impl<S> Continuous<Ray2<S>> for ConvexPolygon<S>
where
    S: BaseFloat,
{
    type Result = Point2<S>;

    /// Ray must be in object space
    fn intersection(&self, ray: &Ray2<S>) -> Option<Point2<S>> {
        for j in 0..self.vertices.len() - 1 {
            let i = if j == 0 {
                self.vertices.len() - 1
            } else {
                j - 1
            };
            let normal = Vector2::new(
                self.vertices[j].y - self.vertices[i].y,
                self.vertices[i].x - self.vertices[j].x,
            );
            // check if edge normal points toward the ray origin
            if ray.direction.dot(normal) < S::zero() {
                // check line ray intersection
                if let point @ Some(_) =
                    ray.intersection(&Line2::new(self.vertices[i], self.vertices[j]))
                {
                    return point;
                }
            }
        }

        None
    }
}

fn support_point<P, T>(vertices: &[P], direction: &P::Diff, transform: &T) -> P
where
    P: EuclideanSpace,
    P::Scalar: BaseFloat,
    T: Transform<P>,
{
    let direction = transform.inverse_transform_vector(*direction).unwrap();

    // figure out where to start, if the direction is negative for the first vertex,
    // go halfway around the polygon
    let mut start_index: i32 = 0;
    let mut max_dot = vertices[0].dot(direction);
    if max_dot < P::Scalar::zero() {
        start_index = vertices.len() as i32 / 2;
        max_dot = dot_index(vertices, start_index, &direction);
    }

    let left_dot = dot_index(vertices, start_index - 1, &direction);
    let right_dot = dot_index(vertices, start_index + 1, &direction);

    // check if start is highest
    let p = if start_index == 0 && max_dot > left_dot && max_dot > right_dot {
        vertices[0]
    } else {
        // figure out iteration direction
        let mut add: i32 = 1;
        let mut previous_dot = left_dot;
        if left_dot > max_dot && left_dot > right_dot {
            add = -1;
            previous_dot = right_dot;
        }

        // iterate
        let mut index = start_index + add;
        let mut current_dot = max_dot;
        if index == vertices.len() as i32 {
            index = 0;
        }
        if index == -1 {
            index = vertices.len() as i32 - 1;
        }
        while index != start_index {
            let next_dot = dot_index(vertices, index + add, &direction);
            if current_dot > previous_dot && current_dot > next_dot {
                break;
            }
            previous_dot = current_dot;
            current_dot = next_dot;
            index += add;
            if index == vertices.len() as i32 {
                index = 0;
            }
            if index == -1 {
                index = vertices.len() as i32 - 1;
            }
        }
        vertices[index as usize]
    };

    transform.transform_point(p)
}

#[inline]
fn dot_index<P>(vertices: &[P], index: i32, direction: &P::Diff) -> P::Scalar
where
    P: EuclideanSpace,
    P::Scalar: BaseFloat,
{
    let index_u = index as usize;
    if index_u == vertices.len() {
        vertices[0]
    } else if index == -1 {
        vertices[vertices.len() - 1]
    } else {
        vertices[index_u]
    }
    .dot(*direction)
}

#[cfg(test)]
mod tests {
    use cgmath::assert_ulps_eq;
    use cgmath::{Basis2, Decomposed, Point2, Rad, Vector2};

    use super::*;
    use {Aabb2, Ray2};

    #[test]
    fn test_support_point() {
        let vertices = vec![
            Point2::new(5., 5.),
            Point2::new(4., 6.),
            Point2::new(3., 7.),
            Point2::new(2., 6.),
            Point2::new(1., 6.),
            Point2::new(0., 5.),
            Point2::new(-1., 4.),
            Point2::new(-3., 3.),
            Point2::new(-6., 1.),
            Point2::new(-5., 0.),
            Point2::new(-4., -1.),
            Point2::new(-2., -3.),
            Point2::new(0., -7.),
            Point2::new(1., -8.),
            Point2::new(2., -5.),
            Point2::new(3., 0.),
            Point2::new(4., 3.),
        ];
        let t = transform(0., 0., 0.);
        let point = support_point(&vertices, &Vector2::new(-1., 0.), &t);
        assert_eq!(Point2::new(-5., 0.), point);

        let point = support_point(&vertices, &Vector2::new(0., -1.), &t);
        assert_eq!(Point2::new(1., -8.), point);

        let point = support_point(&vertices, &Vector2::new(0., 1.), &t);
        assert_eq!(Point2::new(3., 7.), point);

        let point = support_point(&vertices, &Vector2::new(1., 0.), &t);
        assert_eq!(Point2::new(5., 5.), point);
    }

    #[test]
    fn test_bound() {
        let vertices = vec![
            Point2::new(5., 5.),
            Point2::new(4., 6.),
            Point2::new(3., 7.),
            Point2::new(2., 6.),
            Point2::new(1., 6.),
            Point2::new(0., 5.),
            Point2::new(-1., 4.),
            Point2::new(-3., 3.),
            Point2::new(-6., 1.),
            Point2::new(-5., 0.),
            Point2::new(-4., -1.),
            Point2::new(-2., -3.),
            Point2::new(0., -7.),
            Point2::new(1., -8.),
            Point2::new(2., -5.),
            Point2::new(3., 0.),
            Point2::new(4., 3.),
        ];
        let polygon = ConvexPolygon::new(vertices);
        assert_eq!(
            Aabb2::new(Point2::new(-6., -8.), Point2::new(5., 7.)),
            polygon.compute_bound()
        );
    }

    #[test]
    fn test_ray_discrete() {
        let vertices = vec![
            Point2::new(5., 5.),
            Point2::new(4., 6.),
            Point2::new(3., 7.),
            Point2::new(2., 6.),
            Point2::new(1., 6.),
            Point2::new(0., 5.),
            Point2::new(-1., 4.),
            Point2::new(-3., 3.),
            Point2::new(-6., 1.),
            Point2::new(-5., 0.),
            Point2::new(-4., -1.),
            Point2::new(-2., -3.),
            Point2::new(0., -7.),
            Point2::new(1., -8.),
            Point2::new(2., -5.),
            Point2::new(3., 0.),
            Point2::new(4., 3.),
        ];
        let polygon = ConvexPolygon::new(vertices);
        let ray = Ray2::new(Point2::new(0., 10.), Vector2::new(0., -1.));
        assert!(polygon.intersects(&ray));
        let ray = Ray2::new(Point2::new(0., 10.), Vector2::new(0., 1.));
        assert!(!polygon.intersects(&ray));
        let ray = Ray2::new(Point2::new(0., 7.2), Vector2::new(1., 0.));
        assert!(!polygon.intersects(&ray));
        let ray = Ray2::new(Point2::new(0., 6.8), Vector2::new(1., 0.));
        assert!(polygon.intersects(&ray));
    }

    #[test]
    fn test_ray_discrete_transformed() {
        let vertices = vec![
            Point2::new(5., 5.),
            Point2::new(4., 6.),
            Point2::new(3., 7.),
            Point2::new(2., 6.),
            Point2::new(1., 6.),
            Point2::new(0., 5.),
            Point2::new(-1., 4.),
            Point2::new(-3., 3.),
            Point2::new(-6., 1.),
            Point2::new(-5., 0.),
            Point2::new(-4., -1.),
            Point2::new(-2., -3.),
            Point2::new(0., -7.),
            Point2::new(1., -8.),
            Point2::new(2., -5.),
            Point2::new(3., 0.),
            Point2::new(4., 3.),
        ];
        let polygon = ConvexPolygon::new(vertices);
        let t = transform(0., 0., 0.);
        let ray = Ray2::new(Point2::new(0., 10.), Vector2::new(0., -1.));
        assert!(polygon.intersects_transformed(&ray, &t));
        let ray = Ray2::new(Point2::new(0., 10.), Vector2::new(0., 1.));
        assert!(!polygon.intersects_transformed(&ray, &t));
        let ray = Ray2::new(Point2::new(0., 7.2), Vector2::new(1., 0.));
        assert!(!polygon.intersects_transformed(&ray, &t));
        let ray = Ray2::new(Point2::new(0., 6.8), Vector2::new(1., 0.));
        assert!(polygon.intersects_transformed(&ray, &t));
        let t = transform(0., -2., 0.);
        assert!(!polygon.intersects_transformed(&ray, &t));
        let t = transform(0., 0., 0.3);
        assert!(polygon.intersects_transformed(&ray, &t));
    }

    #[test]
    fn test_ray_continuous() {
        let vertices = vec![
            Point2::new(5., 5.),
            Point2::new(4., 6.),
            Point2::new(3., 7.),
            Point2::new(2., 6.),
            Point2::new(1., 6.),
            Point2::new(0., 5.),
            Point2::new(-1., 4.),
            Point2::new(-3., 3.),
            Point2::new(-6., 1.),
            Point2::new(-5., 0.),
            Point2::new(-4., -1.),
            Point2::new(-2., -3.),
            Point2::new(0., -7.),
            Point2::new(1., -8.),
            Point2::new(2., -5.),
            Point2::new(3., 0.),
            Point2::new(4., 3.),
        ];
        let polygon = ConvexPolygon::new(vertices);
        let ray = Ray2::new(Point2::new(0., 10.), Vector2::new(0., -1.));
        assert_eq!(Some(Point2::new(0., 5.)), polygon.intersection(&ray));
        let ray = Ray2::new(Point2::new(0., 10.), Vector2::new(0., 1.));
        assert_eq!(None, polygon.intersection(&ray));
        let ray = Ray2::new(Point2::new(0., 7.2), Vector2::new(1., 0.));
        assert_eq!(None, polygon.intersection(&ray));
        let ray = Ray2::new(Point2::new(0., 6.8), Vector2::new(1., 0.));
        let p = polygon.intersection(&ray).unwrap();
        assert_ulps_eq!(2.8, p.x);
        assert_ulps_eq!(6.8, p.y);
    }

    #[test]
    fn test_ray_continuous_transformed() {
        let vertices = vec![
            Point2::new(5., 5.),
            Point2::new(4., 6.),
            Point2::new(3., 7.),
            Point2::new(2., 6.),
            Point2::new(1., 6.),
            Point2::new(0., 5.),
            Point2::new(-1., 4.),
            Point2::new(-3., 3.),
            Point2::new(-6., 1.),
            Point2::new(-5., 0.),
            Point2::new(-4., -1.),
            Point2::new(-2., -3.),
            Point2::new(0., -7.),
            Point2::new(1., -8.),
            Point2::new(2., -5.),
            Point2::new(3., 0.),
            Point2::new(4., 3.),
        ];
        let t = transform(0., 0., 0.);
        let polygon = ConvexPolygon::new(vertices);
        let ray = Ray2::new(Point2::new(0., 10.), Vector2::new(0., -1.));
        assert_eq!(
            Some(Point2::new(0., 5.)),
            polygon.intersection_transformed(&ray, &t)
        );
        let ray = Ray2::new(Point2::new(0., 10.), Vector2::new(0., 1.));
        assert_eq!(None, polygon.intersection_transformed(&ray, &t));
        let ray = Ray2::new(Point2::new(0., 7.2), Vector2::new(1., 0.));
        assert_eq!(None, polygon.intersection_transformed(&ray, &t));
        let ray = Ray2::new(Point2::new(0., 6.8), Vector2::new(1., 0.));
        let p = polygon.intersection_transformed(&ray, &t).unwrap();
        assert_ulps_eq!(2.8, p.x);
        assert_ulps_eq!(6.8, p.y);
        let t = transform(0., -2., 0.);
        assert_eq!(None, polygon.intersection_transformed(&ray, &t));
        let t = transform(0., 0., 0.3);
        let p = polygon.intersection_transformed(&ray, &t).unwrap();
        assert_ulps_eq!(0.389_133_57, p.x);
        assert_ulps_eq!(6.8, p.y);
    }

    fn transform(dx: f32, dy: f32, rot: f32) -> Decomposed<Vector2<f32>, Basis2<f32>> {
        Decomposed {
            scale: 1.,
            rot: Rotation2::from_angle(Rad(rot)),
            disp: Vector2::new(dx, dy),
        }
    }
}
