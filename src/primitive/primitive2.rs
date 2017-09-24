//! Wrapper enum for 2D primitives

use cgmath::{BaseFloat, Point2, Vector2};
use cgmath::prelude::*;

use {Aabb2, Ray2};
use prelude::*;
use primitive::{Circle, ConvexPolygon, Particle2, Rectangle};

/// Wrapper enum for 2D primitives, that also implements the `Primitive` trait, making it easier
/// to use lots of different primitives in algorithms.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "eders", derive(Serialize, Deserialize))]
pub enum Primitive2<S> {
    /// Particle primitive
    Particle(Particle2<S>),

    /// Circle
    Circle(Circle<S>),

    /// Rectangle
    Rectangle(Rectangle<S>),

    /// Convex polygon with any number of vertices.
    ConvexPolygon(ConvexPolygon<S>),
}


impl<S> Into<Primitive2<S>> for Circle<S> {
    fn into(self) -> Primitive2<S> {
        Primitive2::Circle(self)
    }
}

impl<S> Into<Primitive2<S>> for Rectangle<S> {
    fn into(self) -> Primitive2<S> {
        Primitive2::Rectangle(self)
    }
}

impl<S> Into<Primitive2<S>> for ConvexPolygon<S> {
    fn into(self) -> Primitive2<S> {
        Primitive2::ConvexPolygon(self)
    }
}

impl<S> HasAabb for Primitive2<S>
where
    S: BaseFloat,
{
    type Aabb = Aabb2<S>;

    fn get_bound(&self) -> Aabb2<S> {
        match *self {
            Primitive2::Particle(ref particle) => particle.get_bound(),
            Primitive2::Circle(ref circle) => circle.get_bound(),
            Primitive2::Rectangle(ref rectangle) => rectangle.get_bound(),
            Primitive2::ConvexPolygon(ref polygon) => polygon.get_bound(),
        }
    }
}

impl<S> SupportFunction for Primitive2<S>
where
    S: BaseFloat,
{
    type Point = Point2<S>;

    fn support_point<T>(&self, direction: &Vector2<S>, transform: &T) -> Point2<S>
    where
        T: Transform<Point2<S>>,
    {
        match *self {
            Primitive2::Particle(ref particle) => particle.support_point(direction, transform),
            Primitive2::Circle(ref circle) => circle.support_point(direction, transform),
            Primitive2::Rectangle(ref rectangle) => rectangle.support_point(direction, transform),
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
            Primitive2::Circle(ref circle) => circle.intersects_transformed(ray, transform),
            Primitive2::Rectangle(ref rectangle) => {
                rectangle.intersects_transformed(ray, transform)
            }
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
            Primitive2::Circle(ref circle) => circle.intersection_transformed(ray, transform),
            Primitive2::Rectangle(ref rectangle) => {
                rectangle.intersection_transformed(ray, transform)
            }
            Primitive2::ConvexPolygon(ref polygon) => {
                polygon.intersection_transformed(ray, transform)
            }
        }
    }
}
