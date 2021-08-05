use cgmath::{BaseFloat, InnerSpace, Point2, Transform, Vector2};

use crate::line::Line2;
use crate::traits::{ComputeBound, Primitive};
use crate::Aabb2;

impl<S> Primitive for Line2<S>
where
    S: BaseFloat,
{
    type Point = Point2<S>;

    fn support_point<T>(&self, direction: &Vector2<S>, transform: &T) -> Self::Point
    where
        T: Transform<Self::Point>,
    {
        let direction = transform.inverse_transform_vector(*direction).unwrap();
        let t = direction.dot(self.dest - self.origin);
        if t >= S::zero() {
            transform.transform_point(self.dest)
        } else {
            transform.transform_point(self.origin)
        }
    }
}

impl<S> ComputeBound<Aabb2<S>> for Line2<S>
where
    S: BaseFloat,
{
    fn compute_bound(&self) -> Aabb2<S> {
        Aabb2::new(self.origin, self.dest)
    }
}

#[cfg(test)]
mod tests {

    use super::*;
    use crate::algorithm::minkowski::GJK2;
    use crate::primitive::Rectangle;
    use cgmath::{Basis2, Decomposed, Rad, Rotation2};

    fn transform(x: f32, y: f32, angle: f32) -> Decomposed<Vector2<f32>, Basis2<f32>> {
        Decomposed {
            disp: Vector2::new(x, y),
            rot: Rotation2::from_angle(Rad(angle)),
            scale: 1.,
        }
    }

    #[test]
    fn test_line_rectangle_intersect() {
        let line = Line2::new(Point2::new(0., -1.), Point2::new(0., 1.));
        let rectangle = Rectangle::new(1., 0.2);
        let transform_1 = transform(1., 0., 0.);
        let transform_2 = transform(1.1, 0., 0.);
        let gjk = GJK2::new();
        assert!(gjk
            .intersect(&line, &transform_1, &rectangle, &transform_2)
            .is_some());
    }
}
