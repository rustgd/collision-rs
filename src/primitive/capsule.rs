use cgmath::{BaseFloat, Point3, Vector3};
use cgmath::prelude::*;

use {Aabb3, Ray3};
use prelude::*;

/// Capsule primitive
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "eders", derive(Serialize, Deserialize))]
pub struct Capsule<S> {
    half_height: S,
    radius: S,
}

impl<S> Capsule<S> {
    /// Create a new Capsule
    pub fn new(half_height: S, radius: S) -> Self {
        Self {
            half_height,
            radius,
        }
    }
}

impl<S> SupportFunction for Capsule<S>
    where
        S: BaseFloat,
{
    type Point = Point3<S>;

    fn support_point<T>(&self, direction: &Vector3<S>, transform: &T) -> Point3<S>
        where
            T: Transform<Point3<S>>,
    {
        let direction = transform
            .inverse_transform()
            .unwrap()
            .transform_vector(*direction);

        let mut result = Vector3::zero();
        if direction.y.is_sign_negative() {
            result[1] = - self.half_height;
        } else {
            result[1] = self.half_height;
        }
        transform.transform_point(Point3::from_vec(result + direction.normalize_to(self.radius)))
    }
}

impl<S> HasAabb for Capsule<S>
    where
        S: BaseFloat,
{
    type Aabb = Aabb3<S>;

    fn get_bound(&self) -> Self::Aabb {
        Aabb3::new(
            Point3::new(-self.radius, -self.half_height - self.radius, -self.radius),
            Point3::new(self.radius, self.half_height + self.radius, self.radius),
        )
    }
}

impl<S> Discrete<Ray3<S>> for Capsule<S>
    where
        S: BaseFloat,
{
    fn intersects(&self, _: &Ray3<S>) -> bool {
        false // TODO
    }
}

impl<S> Continuous<Ray3<S>> for Capsule<S>
    where
        S: BaseFloat,
{
    type Result = Point3<S>;

    fn intersection(&self, _: &Ray3<S>) -> Option<Point3<S>> {
        None // TODO
    }
}
