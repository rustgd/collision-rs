//! Rectangle primitive

use cgmath::{Point2, Vector2, BaseFloat};
use cgmath::prelude::*;

use {Aabb2, Ray2};
use prelude::*;
use traits::{ContinuousTransformed, DiscreteTransformed, HasAABB, SupportFunction};

/// Rectangle primitive.
///
/// Have a cached set of corner points to speed up computation.
#[derive(Debug, Clone)]
pub struct Rectangle<S> {
    /// Dimensions of the rectangle
    pub dim: Vector2<S>,
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
        Rectangle {
            dim,
            half_dim: dim / (S::one() + S::one()),
            corners: Self::generate_corners(&dim),
        }
    }

    fn generate_corners(dimensions: &Vector2<S>) -> Vec<Point2<S>> {
        let two = S::one() + S::one();
        vec![
            Point2::new(dimensions.x / two, dimensions.y / two),
            Point2::new(-dimensions.x / two, dimensions.y / two),
            Point2::new(-dimensions.x / two, -dimensions.y / two),
            Point2::new(dimensions.x / two, -dimensions.y / two),
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
        ::primitive::util::get_max_point(&self.corners, direction, transform)
    }
}

impl<S> HasAABB for Rectangle<S>
where
    S: BaseFloat,
{
    type Aabb = Aabb2<S>;

    fn get_bound(&self) -> Aabb2<S> {
        Aabb2::new(
            Point2::from_vec(-self.half_dim),
            Point2::from_vec(self.half_dim),
        )
    }
}

impl<S> DiscreteTransformed<Ray2<S>> for Rectangle<S>
where
    S: BaseFloat,
{
    type Point = Point2<S>;

    fn intersects_transformed<T>(&self, ray: &Ray2<S>, transform: &T) -> bool
    where
        T: Transform<Point2<S>>,
    {
        self.intersects(&ray.transform(transform.inverse_transform().unwrap()))
    }
}

impl<S> ContinuousTransformed<Ray2<S>> for Rectangle<S>
where
    S: BaseFloat,
{
    type Point = Point2<S>;
    type Result = Point2<S>;

    fn intersection_transformed<T>(&self, ray: &Ray2<S>, transform: &T) -> Option<Point2<S>>
    where
        T: Transform<Point2<S>>,
    {
        self.intersection(&ray.transform(transform.inverse_transform().unwrap()))
            .map(|p| transform.transform_point(p))
    }
}

impl<S> Discrete<Ray2<S>> for Rectangle<S>
where
    S: BaseFloat,
{
    /// Ray must be in object space of the rectangle
    fn intersects(&self, ray: &Ray2<S>) -> bool {
        let min = Point2::new(-self.half_dim.x, -self.half_dim.y);
        let max = Point2::new(self.half_dim.x, self.half_dim.y);

        let mut tmin = S::neg_infinity();
        let mut tmax = S::infinity();

        if ray.direction.x != S::zero() {
            let tx1 = (min.x - ray.origin.x) / ray.direction.x;
            let tx2 = (max.x - ray.origin.x) / ray.direction.x;
            tmin = tmin.max(tx1.min(tx2));
            tmax = tmax.min(tx1.max(tx2));
        } else if ray.origin.x <= min.x || ray.origin.x >= max.x {
            return false;
        }

        if ray.direction.y != S::zero() {
            let ty1 = (min.y - ray.origin.y) / ray.direction.y;
            let ty2 = (max.y - ray.origin.y) / ray.direction.y;
            tmin = tmin.max(ty1.min(ty2));
            tmax = tmax.min(ty1.max(ty2));
        } else if ray.origin.y <= min.y || ray.origin.y >= max.y {
            return false;
        }

        tmax >= tmin && (tmin >= S::zero() || tmax >= S::zero())
    }
}

impl<S> Continuous<Ray2<S>> for Rectangle<S>
where
    S: BaseFloat,
{
    type Result = Point2<S>;

    /// Ray must be in object space of the rectangle
    fn intersection(&self, ray: &Ray2<S>) -> Option<Point2<S>> {
        let min = Point2::new(-self.half_dim.x, -self.half_dim.y);
        let max = Point2::new(self.half_dim.x, self.half_dim.y);
        let mut tmin = S::neg_infinity();
        let mut tmax = S::infinity();

        if ray.direction.x != S::zero() {
            let tx1 = (min.x - ray.origin.x) / ray.direction.x;
            let tx2 = (max.x - ray.origin.x) / ray.direction.x;
            tmin = tmin.max(tx1.min(tx2));
            tmax = tmax.min(tx1.max(tx2));
        } else if ray.origin.x <= min.x || ray.origin.x >= max.x {
            return None;
        }

        if ray.direction.y != S::zero() {
            let ty1 = (min.y - ray.origin.y) / ray.direction.y;
            let ty2 = (max.y - ray.origin.y) / ray.direction.y;
            tmin = tmin.max(ty1.min(ty2));
            tmax = tmax.min(ty1.max(ty2));
        } else if ray.origin.y <= min.y || ray.origin.y >= max.y {
            return None;
        }

        if (tmin < S::zero() && tmax < S::zero()) || tmax < tmin {
            None
        } else {
            let t = if tmin >= S::zero() { tmin } else { tmax };
            Some(ray.origin + ray.direction * t)
        }
    }
}

#[cfg(test)]
mod tests {
    use cgmath::{Point2, Vector2, Rad, Basis2, Decomposed};

    use super::*;

    #[test]
    fn test_rectangle_bound() {
        let r = Rectangle::new(10., 10.);
        assert_eq!(bound(-5., -5., 5., 5.), r.get_bound())
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
