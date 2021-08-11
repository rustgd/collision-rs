use crate::prelude::*;

/// Broad phase collision detection brute force implementation.
///
/// Will simply do bounding box intersection tests for all shape combinations.
#[derive(Debug, Default, Copy, Clone)]
pub struct BruteForce;

impl BruteForce {
    /// Find all potentially colliding pairs of shapes.
    ///
    /// ## Parameters
    ///
    /// - `shapes`: Shapes to do find potential collisions for
    ///
    /// ## Returns
    ///
    /// Returns tuples with the into the shapes list, of all potentially colliding pairs.
    /// The first value in the tuple will always be first in the list.
    pub fn find_collider_pairs<A>(&self, shapes: &[A]) -> Vec<(usize, usize)>
    where
        A: HasBound,
        A::Bound: Discrete<A::Bound>,
    {
        let mut pairs = Vec::default();
        if shapes.len() <= 1 {
            return pairs;
        }

        for left_index in 0..(shapes.len() - 1) {
            let left = &shapes[left_index];
            for right_index in (left_index + 1)..shapes.len() {
                if left.bound().intersects(shapes[right_index].bound()) {
                    pairs.push((left_index, right_index));
                }
            }
        }
        pairs
    }
}

#[cfg(test)]
mod tests {
    use cgmath::Point2;

    use super::*;
    use crate::Aabb2;

    #[derive(Debug, Clone, PartialEq)]
    pub struct BroadCollisionInfo2 {
        /// The id
        pub id: u32,

        /// The bounding volume
        pub bound: Aabb2<f32>,
        index: usize,
    }

    impl BroadCollisionInfo2 {
        /// Create a new collision info
        pub fn new(id: u32, bound: Aabb2<f32>) -> Self {
            Self {
                id,
                bound,
                index: 0,
            }
        }
    }

    impl HasBound for BroadCollisionInfo2 {
        type Bound = Aabb2<f32>;

        fn bound(&self) -> &Aabb2<f32> {
            &self.bound
        }
    }

    #[test]
    fn no_intersection_for_miss() {
        let left = coll(1, 8., 8., 10., 11.);

        let right = coll(2, 12., 13., 18., 18.);

        let brute = BruteForce::default();
        let potentials = brute.find_collider_pairs(&[left, right]);
        assert_eq!(0, potentials.len());
    }

    #[test]
    fn no_intersection_for_miss_unsorted() {
        let left = coll(1, 8., 8., 10., 11.);

        let right = coll(2, 12., 13., 18., 18.);

        let brute = BruteForce::default();
        let potentials = brute.find_collider_pairs(&[right, left]);
        assert_eq!(0, potentials.len());
    }

    #[test]
    fn intersection_for_hit() {
        let left = coll(1, 8., 8., 10., 11.);

        let right = coll(2, 9., 10., 18., 18.);

        let brute = BruteForce::default();
        let potentials = brute.find_collider_pairs(&[left, right]);
        assert_eq!(1, potentials.len());
        assert_eq!((0, 1), potentials[0]);
    }

    #[test]
    fn intersection_for_hit_unsorted() {
        let left = coll(1, 8., 8., 10., 11.);

        let right = coll(222, 9., 10., 18., 18.);

        let brute = BruteForce::default();
        let potentials = brute.find_collider_pairs(&[right, left]);
        assert_eq!(1, potentials.len());
        assert_eq!((0, 1), potentials[0]);
    }

    // util
    fn coll(id: u32, min_x: f32, min_y: f32, max_x: f32, max_y: f32) -> BroadCollisionInfo2 {
        BroadCollisionInfo2::new(id, bound(min_x, min_y, max_x, max_y))
    }

    fn bound(min_x: f32, min_y: f32, max_x: f32, max_y: f32) -> Aabb2<f32> {
        Aabb2::new(Point2::new(min_x, min_y), Point2::new(max_x, max_y))
    }
}
