//! Rectangle primitive

use cgmath::{BaseFloat, Point2, Vector2};
use cgmath::prelude::*;

use {Aabb2, Ray2};
use prelude::*;
use primitive::util::get_max_point;

/// Rectangle primitive.
///
/// Have a cached set of corner points to speed up computation.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "eders", derive(Serialize, Deserialize))]
pub struct Rectangle<S> {
    /// Dimensions of the rectangle
    dim: Vector2<S>,
    half_dim: Vector2<S>,
    corners: Vec<Point2<S>>,
}

impl<S> Rectangle<S>
where
    S: BaseFloat,
{
    /// Create a new rectangle primitive from component dimensions
    pub fn new(dim_x: S, dim_y: S) -> Self {
        Self::new_impl(Vector2::new(dim_x, dim_y))
    }

    /// Create a new rectangle primitive from a vector of component dimensions
    pub fn new_impl(dim: Vector2<S>) -> Self {
        let half_dim = dim / (S::one() + S::one());
        Rectangle {
            dim,
            half_dim,
            corners: Self::generate_corners(&half_dim),
        }
    }

    fn generate_corners(half_dim: &Vector2<S>) -> Vec<Point2<S>> {
        vec![
            Point2::new(half_dim.x, half_dim.y),
            Point2::new(-half_dim.x, half_dim.y),
            Point2::new(-half_dim.x, -half_dim.y),
            Point2::new(half_dim.x, -half_dim.y),
        ]
    }
}

impl<S> SupportFunction for Rectangle<S>
where
    S: BaseFloat,
{
    type Point = Point2<S>;

    fn support_point<T>(&self, direction: &Vector2<S>, transform: &T) -> Point2<S>
    where
        T: Transform<Point2<S>>,
    {
        get_max_point(self.corners.iter(), direction, transform)
    }
}

impl<S> ComputeBound<Aabb2<S>> for Rectangle<S>
where
    S: BaseFloat,
{
    fn compute_bound(&self) -> Aabb2<S> {
        Aabb2::new(
            Point2::from_vec(-self.half_dim),
            Point2::from_vec(self.half_dim),
        )
    }
}

impl<S> Discrete<Ray2<S>> for Rectangle<S>
where
    S: BaseFloat,
{
    /// Ray must be in object space of the rectangle
    fn intersects(&self, ray: &Ray2<S>) -> bool {
        self.compute_bound().intersects(ray)
    }
}

impl<S> Continuous<Ray2<S>> for Rectangle<S>
where
    S: BaseFloat,
{
    type Result = Point2<S>;

    /// Ray must be in object space of the rectangle
    fn intersection(&self, ray: &Ray2<S>) -> Option<Point2<S>> {
        self.compute_bound().intersection(ray)
    }
}

#[cfg(test)]
mod tests {
    use cgmath::{Basis2, Decomposed, Point2, Rad, Vector2};

    use super::*;

    #[test]
    fn test_rectangle_bound() {
        let r = Rectangle::new(10., 10.);
        assert_eq!(bound(-5., -5., 5., 5.), r.compute_bound())
    }

    #[test]
    fn test_rectangle_ray_discrete() {
        let rectangle = Rectangle::new(10., 10.);
        let ray = Ray2::new(Point2::new(20., 0.), Vector2::new(-1., 0.));
        assert!(rectangle.intersects(&ray));
        let ray = Ray2::new(Point2::new(20., 0.), Vector2::new(0., 1.));
        assert!(!rectangle.intersects(&ray));
    }

    #[test]
    fn test_rectangle_ray_discrete_transformed() {
        let rectangle = Rectangle::new(10., 10.);
        let ray = Ray2::new(Point2::new(20., 0.), Vector2::new(-1., 0.));
        let t = transform(0., 0., 0.);
        assert!(rectangle.intersects_transformed(&ray, &t));
        let ray = Ray2::new(Point2::new(20., 0.), Vector2::new(-1., 0.));
        let t = transform(0., 20., 0.);
        assert!(!rectangle.intersects_transformed(&ray, &t));
    }

    #[test]
    fn test_rectangle_ray_continuous() {
        let rectangle = Rectangle::new(10., 10.);
        let ray = Ray2::new(Point2::new(20., 0.), Vector2::new(-1., 0.));
        assert_eq!(Some(Point2::new(5., 0.)), rectangle.intersection(&ray));
        let ray = Ray2::new(Point2::new(20., 0.), Vector2::new(0., 1.));
        assert_eq!(None, rectangle.intersection(&ray));
    }

    #[test]
    fn test_rectangle_ray_continuous_transformed() {
        let rectangle = Rectangle::new(10., 10.);
        let ray = Ray2::new(Point2::new(20., 0.), Vector2::new(-1., 0.));
        let t = transform(0., 0., 0.);
        assert_eq!(
            Some(Point2::new(5., 0.)),
            rectangle.intersection_transformed(&ray, &t)
        );
        let ray = Ray2::new(Point2::new(20., 0.), Vector2::new(-1., 0.));
        let t = transform(0., 20., 0.);
        assert_eq!(None, rectangle.intersection_transformed(&ray, &t));
        let t = transform(0., 0., 0.3);
        let p = rectangle.intersection_transformed(&ray, &t).unwrap();
        assert_ulps_eq!(5.233758, p.x);
        assert_ulps_eq!(0., p.y);
    }

    // util
    fn transform(dx: f32, dy: f32, rot: f32) -> Decomposed<Vector2<f32>, Basis2<f32>> {
        Decomposed {
            scale: 1.,
            rot: Rotation2::from_angle(Rad(rot)),
            disp: Vector2::new(dx, dy),
        }
    }

    fn bound(min_x: f32, min_y: f32, max_x: f32, max_y: f32) -> Aabb2<f32> {
        Aabb2::new(Point2::new(min_x, min_y), Point2::new(max_x, max_y))
    }
}
