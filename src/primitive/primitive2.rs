//! Wrapper enum for 2D primitives

use cgmath::prelude::*;
use cgmath::{BaseFloat, Point2, Vector2};

use crate::prelude::*;
use crate::primitive::{Circle, ConvexPolygon, Particle2, Rectangle, Square};
use crate::{Aabb2, Line2, Ray2};

/// Wrapper enum for 2D primitives, that also implements the `Primitive` trait, making it easier
/// to use many different primitives in algorithms.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(
    feature = "serde",
    derive(Serialize, Deserialize),
    serde(crate = "serde_crate")
)]
pub enum Primitive2<S: cgmath::BaseNum> {
    /// Particle
    Particle(Particle2<S>),
    /// Line
    Line(Line2<S>),
    /// Circle
    Circle(Circle<S>),
    /// Rectangle
    Rectangle(Rectangle<S>),
    /// Square
    Square(Square<S>),
    /// Convex polygon with any number of vertices.
    ConvexPolygon(ConvexPolygon<S>),
}

impl<S: cgmath::BaseNum> From<Particle2<S>> for Primitive2<S> {
    fn from(particle: Particle2<S>) -> Primitive2<S> {
        Primitive2::Particle(particle)
    }
}

impl<S: cgmath::BaseNum> From<Line2<S>> for Primitive2<S> {
    fn from(line: Line2<S>) -> Primitive2<S> {
        Primitive2::Line(line)
    }
}

impl<S: cgmath::BaseNum> From<Circle<S>> for Primitive2<S> {
    fn from(circle: Circle<S>) -> Primitive2<S> {
        Primitive2::Circle(circle)
    }
}

impl<S: cgmath::BaseNum> From<Rectangle<S>> for Primitive2<S> {
    fn from(rectangle: Rectangle<S>) -> Primitive2<S> {
        Primitive2::Rectangle(rectangle)
    }
}

impl<S: cgmath::BaseNum> From<Square<S>> for Primitive2<S> {
    fn from(rectangle: Square<S>) -> Primitive2<S> {
        Primitive2::Square(rectangle)
    }
}

impl<S: cgmath::BaseNum> From<ConvexPolygon<S>> for Primitive2<S> {
    fn from(polygon: ConvexPolygon<S>) -> Primitive2<S> {
        Primitive2::ConvexPolygon(polygon)
    }
}

impl<S> ComputeBound<Aabb2<S>> for Primitive2<S>
where
    S: BaseFloat,
{
    fn compute_bound(&self) -> Aabb2<S> {
        match *self {
            Primitive2::Particle(_) => Aabb2::zero(),
            Primitive2::Line(ref line) => line.compute_bound(),
            Primitive2::Circle(ref circle) => circle.compute_bound(),
            Primitive2::Rectangle(ref rectangle) => rectangle.compute_bound(),
            Primitive2::Square(ref square) => square.compute_bound(),
            Primitive2::ConvexPolygon(ref polygon) => polygon.compute_bound(),
        }
    }
}

impl<S> Primitive for Primitive2<S>
where
    S: BaseFloat,
{
    type Point = Point2<S>;

    fn support_point<T>(&self, direction: &Vector2<S>, transform: &T) -> Point2<S>
    where
        T: Transform<Point2<S>>,
    {
        match *self {
            Primitive2::Particle(_) => transform.transform_point(Point2::origin()),
            Primitive2::Line(ref line) => line.support_point(direction, transform),
            Primitive2::Circle(ref circle) => circle.support_point(direction, transform),
            Primitive2::Rectangle(ref rectangle) => rectangle.support_point(direction, transform),
            Primitive2::Square(ref square) => square.support_point(direction, transform),
            Primitive2::ConvexPolygon(ref polygon) => polygon.support_point(direction, transform),
        }
    }
}

impl<S> DiscreteTransformed<Ray2<S>> for Primitive2<S>
where
    S: BaseFloat,
{
    type Point = Point2<S>;

    fn intersects_transformed<T>(&self, ray: &Ray2<S>, transform: &T) -> bool
    where
        T: Transform<Self::Point>,
    {
        match *self {
            Primitive2::Particle(ref particle) => particle.intersects_transformed(ray, transform),
            Primitive2::Line(ref line) => line.intersects_transformed(ray, transform),
            Primitive2::Circle(ref circle) => circle.intersects_transformed(ray, transform),
            Primitive2::Rectangle(ref rectangle) => {
                rectangle.intersects_transformed(ray, transform)
            }
            Primitive2::Square(ref square) => square.intersects_transformed(ray, transform),
            Primitive2::ConvexPolygon(ref polygon) => {
                polygon.intersects_transformed(ray, transform)
            }
        }
    }
}

impl<S> ContinuousTransformed<Ray2<S>> for Primitive2<S>
where
    S: BaseFloat,
{
    type Point = Point2<S>;
    type Result = Point2<S>;

    fn intersection_transformed<T>(&self, ray: &Ray2<S>, transform: &T) -> Option<Point2<S>>
    where
        T: Transform<Point2<S>>,
    {
        match *self {
            Primitive2::Particle(ref particle) => particle.intersection_transformed(ray, transform),
            Primitive2::Line(ref line) => line.intersection_transformed(ray, transform),
            Primitive2::Circle(ref circle) => circle.intersection_transformed(ray, transform),
            Primitive2::Rectangle(ref rectangle) => {
                rectangle.intersection_transformed(ray, transform)
            }
            Primitive2::Square(ref square) => square.intersection_transformed(ray, transform),
            Primitive2::ConvexPolygon(ref polygon) => {
                polygon.intersection_transformed(ray, transform)
            }
        }
    }
}
