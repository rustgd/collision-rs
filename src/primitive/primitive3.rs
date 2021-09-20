//! Wrapper enum for 3D primitives

use cgmath::prelude::*;
use cgmath::{BaseFloat, Point3, Vector3};

use crate::prelude::*;
use crate::primitive::{
    Capsule, ConvexPolyhedron, Cube, Cuboid, Cylinder, Particle3, Quad, Sphere,
};
use crate::{Aabb3, Ray3};

/// Wrapper enum for 3D primitives, that also implements the `Primitive` trait, making it easier
/// to use many different primitives in algorithms.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(
    feature = "serde",
    derive(Serialize, Deserialize),
    serde(crate = "serde_crate")
)]
pub enum Primitive3<S>
where
    S: BaseFloat,
{
    /// Particle
    Particle(Particle3<S>),
    /// Rectangular plane
    Quad(Quad<S>),
    /// Sphere
    Sphere(Sphere<S>),
    /// Cuboid
    Cuboid(Cuboid<S>),
    /// Cube
    Cube(Cube<S>),
    /// Cylinder
    Cylinder(Cylinder<S>),
    /// Capsule
    Capsule(Capsule<S>),
    /// Convex polyhedron with any number of vertices/faces
    ConvexPolyhedron(ConvexPolyhedron<S>),
}

impl<S> From<Particle3<S>> for Primitive3<S>
where
    S: BaseFloat,
{
    fn from(particle: Particle3<S>) -> Primitive3<S> {
        Primitive3::Particle(particle)
    }
}

impl<S> From<Quad<S>> for Primitive3<S>
where
    S: BaseFloat,
{
    fn from(quad: Quad<S>) -> Self {
        Primitive3::Quad(quad)
    }
}

impl<S> From<Sphere<S>> for Primitive3<S>
where
    S: BaseFloat,
{
    fn from(sphere: Sphere<S>) -> Primitive3<S> {
        Primitive3::Sphere(sphere)
    }
}

impl<S> From<Cube<S>> for Primitive3<S>
where
    S: BaseFloat,
{
    fn from(cuboid: Cube<S>) -> Primitive3<S> {
        Primitive3::Cube(cuboid)
    }
}

impl<S> From<Cuboid<S>> for Primitive3<S>
where
    S: BaseFloat,
{
    fn from(cuboid: Cuboid<S>) -> Primitive3<S> {
        Primitive3::Cuboid(cuboid)
    }
}

impl<S> From<Cylinder<S>> for Primitive3<S>
where
    S: BaseFloat,
{
    fn from(cylinder: Cylinder<S>) -> Primitive3<S> {
        Primitive3::Cylinder(cylinder)
    }
}

impl<S> From<Capsule<S>> for Primitive3<S>
where
    S: BaseFloat,
{
    fn from(capsule: Capsule<S>) -> Primitive3<S> {
        Primitive3::Capsule(capsule)
    }
}

impl<S> From<ConvexPolyhedron<S>> for Primitive3<S>
where
    S: BaseFloat,
{
    fn from(polyhedron: ConvexPolyhedron<S>) -> Primitive3<S> {
        Primitive3::ConvexPolyhedron(polyhedron)
    }
}

impl<S> ComputeBound<Aabb3<S>> for Primitive3<S>
where
    S: BaseFloat,
{
    fn compute_bound(&self) -> Aabb3<S> {
        match *self {
            Primitive3::Particle(_) => Aabb3::zero(),
            Primitive3::Quad(ref quad) => quad.compute_bound(),
            Primitive3::Cuboid(ref cuboid) => cuboid.compute_bound(),
            Primitive3::Cube(ref cuboid) => cuboid.compute_bound(),
            Primitive3::Sphere(ref sphere) => sphere.compute_bound(),
            Primitive3::Cylinder(ref cylinder) => cylinder.compute_bound(),
            Primitive3::Capsule(ref capsule) => capsule.compute_bound(),
            Primitive3::ConvexPolyhedron(ref polyhedron) => polyhedron.compute_bound(),
        }
    }
}

impl<S> ComputeBound<crate::volume::Sphere<S>> for Primitive3<S>
where
    S: BaseFloat,
{
    fn compute_bound(&self) -> crate::volume::Sphere<S> {
        match *self {
            Primitive3::Particle(_) => crate::volume::Sphere {
                center: Point3::origin(),
                radius: S::zero(),
            },
            Primitive3::Quad(ref quad) => quad.compute_bound(),
            Primitive3::Cuboid(ref cuboid) => cuboid.compute_bound(),
            Primitive3::Cube(ref cuboid) => cuboid.compute_bound(),
            Primitive3::Sphere(ref sphere) => sphere.compute_bound(),
            Primitive3::Cylinder(ref cylinder) => cylinder.compute_bound(),
            Primitive3::Capsule(ref capsule) => capsule.compute_bound(),
            Primitive3::ConvexPolyhedron(ref polyhedron) => polyhedron.compute_bound(),
        }
    }
}

impl<S> Primitive for Primitive3<S>
where
    S: BaseFloat,
{
    type Point = Point3<S>;

    fn support_point<T>(&self, direction: &Vector3<S>, transform: &T) -> Point3<S>
    where
        T: Transform<Point3<S>>,
    {
        match *self {
            Primitive3::Particle(_) => transform.transform_point(Point3::origin()),
            Primitive3::Quad(ref quad) => quad.support_point(direction, transform),
            Primitive3::Sphere(ref sphere) => sphere.support_point(direction, transform),
            Primitive3::Cuboid(ref cuboid) => cuboid.support_point(direction, transform),
            Primitive3::Cube(ref cuboid) => cuboid.support_point(direction, transform),
            Primitive3::Cylinder(ref cylinder) => cylinder.support_point(direction, transform),
            Primitive3::Capsule(ref capsule) => capsule.support_point(direction, transform),
            Primitive3::ConvexPolyhedron(ref polyhedron) => {
                polyhedron.support_point(direction, transform)
            }
        }
    }
}

impl<S> DiscreteTransformed<Ray3<S>> for Primitive3<S>
where
    S: BaseFloat,
{
    type Point = Point3<S>;

    fn intersects_transformed<T>(&self, ray: &Ray3<S>, transform: &T) -> bool
    where
        T: Transform<Self::Point>,
    {
        match *self {
            Primitive3::Particle(ref particle) => particle.intersects_transformed(ray, transform),
            Primitive3::Quad(ref quad) => quad.intersects_transformed(ray, transform),
            Primitive3::Sphere(ref sphere) => sphere.intersects_transformed(ray, transform),
            Primitive3::Cuboid(ref cuboid) => cuboid.intersects_transformed(ray, transform),
            Primitive3::Cube(ref cuboid) => cuboid.intersects_transformed(ray, transform),
            Primitive3::Cylinder(ref cylinder) => cylinder.intersects_transformed(ray, transform),
            Primitive3::Capsule(ref capsule) => capsule.intersects_transformed(ray, transform),
            Primitive3::ConvexPolyhedron(ref polyhedron) => {
                polyhedron.intersects_transformed(ray, transform)
            }
        }
    }
}

impl<S> ContinuousTransformed<Ray3<S>> for Primitive3<S>
where
    S: BaseFloat,
{
    type Point = Point3<S>;
    type Result = Point3<S>;

    fn intersection_transformed<T>(&self, ray: &Ray3<S>, transform: &T) -> Option<Point3<S>>
    where
        T: Transform<Point3<S>>,
    {
        match *self {
            Primitive3::Particle(ref particle) => particle.intersection_transformed(ray, transform),
            Primitive3::Quad(ref quad) => quad.intersection_transformed(ray, transform),
            Primitive3::Sphere(ref sphere) => sphere.intersection_transformed(ray, transform),
            Primitive3::Cuboid(ref cuboid) => cuboid.intersection_transformed(ray, transform),
            Primitive3::Cube(ref cuboid) => cuboid.intersection_transformed(ray, transform),
            Primitive3::Cylinder(ref cylinder) => cylinder.intersection_transformed(ray, transform),
            Primitive3::Capsule(ref capsule) => capsule.intersection_transformed(ray, transform),
            Primitive3::ConvexPolyhedron(ref polyhedron) => {
                polyhedron.intersection_transformed(ray, transform)
            }
        }
    }
}
