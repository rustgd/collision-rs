//! Wrapper enum for 3D primitives

use cgmath::{BaseFloat, Point3, Vector3};
use cgmath::prelude::*;

use prelude::*;
use {Ray3, Aabb3};
use primitive::{Particle3, Sphere, Cuboid, ConvexPolyhedron};

#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "eders", derive(Serialize, Deserialize))]
pub enum Primitive3<S> {
    Particle(Particle3<S>),
    Sphere(Sphere<S>),
    Cuboid(Cuboid<S>),
    ConvexPolyhedron(ConvexPolyhedron<S>),
}