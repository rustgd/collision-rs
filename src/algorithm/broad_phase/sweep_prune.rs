use std::cmp::Ordering;

use num::NumCast;

use self::variance::{Variance, Variance2, Variance3};
use super::HasBound;
use prelude::*;

/// Broad phase sweep and prune algorithm for 2D, see
/// [SweepAndPrune](struct.SweepAndPrune.html) for more information.
pub type SweepAndPrune2<S> = SweepAndPrune<Variance2<S>>;

/// Broad phase sweep and prune algorithm for 3D, see
/// [SweepAndPrune](struct.SweepAndPrune.html) for more information.
pub type SweepAndPrune3<S> = SweepAndPrune<Variance3<S>>;

/// Sweep and prune broad phase collision detection algorithm.
///
/// Will sort the bounding boxes of the collision world along some axis, and will then sweep the
/// sorted list, and compare the bounds along the sweep axis, adding them to an active list when
/// they are encountered, and removing them from the active list when the end extent is passed.
///
/// Any shape pairs found by the base algorithm, will then do a bounding box intersection test,
/// before adding to the resulting pairs list.
///
/// # Type parameters:
///
/// - `V`: Variance type used for computing what axis to use on the next iteration. Should be either
///        `Variance2` or `Variance3`.
pub struct SweepAndPrune<V> {
    sweep_axis: usize,
    variance: V,
}

impl<V> SweepAndPrune<V>
where
    V: Variance,
{
    /// Create a new sweep and prune algorithm, will use the X axis as the first sweep axis
    pub fn new() -> Self {
        Self::with_sweep_axis(0)
    }

    /// Create a new sweep and prune algorithm, starting with the given axis as the first sweep axis
    pub fn with_sweep_axis(sweep_axis: usize) -> Self {
        Self {
            sweep_axis,
            variance: V::new(),
        }
    }

    /// Get sweep axis
    pub fn get_sweep_axis(&self) -> usize {
        self.sweep_axis
    }

    /// Find all potentially colliding pairs of shapes
    ///
    /// ## Parameters
    ///
    /// - `shapes`: Shapes to do find potential collisions for
    ///
    /// ## Returns
    ///
    /// Returns tuples with indices into the shapes list, of all potentially colliding pairs.
    /// The first value in the tuple will always be first in the list.
    ///
    /// ## Side effects:
    ///
    /// The shapes list might have been resorted. The indices in the return values will be for the
    /// sorted list.
    pub fn find_collider_pairs<A>(&mut self, shapes: &mut [A]) -> Vec<(usize, usize)>
    where
        A: HasBound,
        A::Bound: Aabb + Discrete<A::Bound>,
        V: Variance<Bound = A::Bound>,
    {
        let mut pairs = Vec::default();
        if shapes.len() <= 1 {
            return pairs;
        }

        shapes.sort_by(|a, b| {
            let cmp_min = a.get_bound().min()[self.sweep_axis]
                .partial_cmp(&b.get_bound().min()[self.sweep_axis]);
            match cmp_min {
                Some(Ordering::Equal) => {
                    a.get_bound().max()[self.sweep_axis]
                        .partial_cmp(&b.get_bound().max()[self.sweep_axis])
                        .unwrap_or(Ordering::Equal)
                }
                None => Ordering::Equal,
                Some(order) => order,
            }
        });

        self.variance.clear();
        self.variance.add_bound(&shapes[0].get_bound());

        let mut active = vec![0];
        // Remember that the index here will be the index of the iterator, which starts at index 1
        // in the shapes list, so the real shape index is + 1
        for (index, shape) in shapes[1..].iter().enumerate() {
            let shape_index = index + 1;
            // for all currently active bounds, go through and remove any that are to the left of
            // the current bound
            active.retain(|active_index| {
                shapes[*active_index].get_bound().max()[self.sweep_axis]
                    >= shape.get_bound().min()[self.sweep_axis]
            });

            // all shapes in the active list are potential hits, do a real bound intersection test
            // for those, and add to pairs if the bounds intersect.
            for active_index in &active {
                if shapes[*active_index]
                    .get_bound()
                    .intersects(&shape.get_bound())
                {
                    pairs.push((*active_index, shape_index));
                }
            }

            // current bound should be active for the next iteration
            active.push(shape_index);

            // update variance
            self.variance.add_bound(&shape.get_bound());
        }

        // compute sweep axis for the next iteration
        let (axis, _) = self.variance
            .compute_axis(NumCast::from(shapes.len()).unwrap());
        self.sweep_axis = axis;

        pairs
    }
}

mod variance {
    use {Aabb, Aabb2, Aabb3};
    use cgmath::{BaseFloat, Vector2, Vector3};
    use cgmath::prelude::*;

    /// Trait for variance calculation in sweep and prune algorithm
    pub trait Variance {
        /// Point type
        type Bound: Aabb;

        /// Create new variance object
        fn new() -> Self;

        /// Clear variance sums
        fn clear(&mut self);

        /// Add an extent to the variance sums
        fn add_bound(&mut self, bound: &Self::Bound);

        /// Compute the sweep axis based on the internal values
        fn compute_axis(
            &self,
            n: <Self::Bound as Aabb>::Scalar,
        ) -> (usize, <Self::Bound as Aabb>::Scalar);
    }

    /// Variance for 2D sweep and prune
    #[derive(Debug)]
    pub struct Variance2<S> {
        csum: Vector2<S>,
        csumsq: Vector2<S>,
    }

    impl<S> Variance for Variance2<S>
    where
        S: BaseFloat,
    {
        type Bound = Aabb2<S>;

        fn new() -> Self {
            Self {
                csum: Vector2::zero(),
                csumsq: Vector2::zero(),
            }
        }

        fn clear(&mut self) {
            self.csum = Vector2::zero();
            self.csumsq = Vector2::zero();
        }

        #[inline]
        fn add_bound(&mut self, bound: &Aabb2<S>) {
            let min_vec = bound.min().to_vec();
            let max_vec = bound.max().to_vec();
            let sum = min_vec.add_element_wise(max_vec);
            let c = sum / (S::one() + S::one());
            self.csum.add_element_wise(c);
            self.csumsq.add_element_wise(c.mul_element_wise(c));
        }

        #[inline]
        fn compute_axis(&self, n: S) -> (usize, S) {
            let square_n = self.csum.mul_element_wise(self.csum) / n;
            let variance = self.csumsq.sub_element_wise(square_n);
            let mut sweep_axis = 0;
            let mut sweep_variance = variance[0];
            for i in 1..2 {
                let v = variance[i];
                if v > sweep_variance {
                    sweep_axis = i;
                    sweep_variance = v;
                }
            }
            (sweep_axis, sweep_variance)
        }
    }

    /// Variance for 3D sweep and prune
    #[derive(Debug)]
    pub struct Variance3<S> {
        csum: Vector3<S>,
        csumsq: Vector3<S>,
    }

    impl<S> Variance for Variance3<S>
    where
        S: BaseFloat,
    {
        type Bound = Aabb3<S>;

        fn new() -> Self {
            Self {
                csum: Vector3::zero(),
                csumsq: Vector3::zero(),
            }
        }

        fn clear(&mut self) {
            self.csum = Vector3::zero();
            self.csumsq = Vector3::zero();
        }

        #[inline]
        fn add_bound(&mut self, bound: &Aabb3<S>) {
            let min_vec = bound.min().to_vec();
            let max_vec = bound.max().to_vec();
            let sum = min_vec.add_element_wise(max_vec);
            let c = sum / (S::one() + S::one());
            self.csum.add_element_wise(c);
            self.csumsq.add_element_wise(c.mul_element_wise(c));
        }

        #[inline]
        fn compute_axis(&self, n: S) -> (usize, S) {
            let square_n = self.csum.mul_element_wise(self.csum) / n;
            let variance = self.csumsq.sub_element_wise(square_n);
            let mut sweep_axis = 0;
            let mut sweep_variance = variance[0];
            for i in 1..3 {
                let v = variance[i];
                if v > sweep_variance {
                    sweep_axis = i;
                    sweep_variance = v;
                }
            }
            (sweep_axis, sweep_variance)
        }
    }
}

#[cfg(test)]
mod tests {
    use cgmath::Point2;

    use super::*;
    use Aabb2;

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

        fn get_bound(&self) -> &Aabb2<f32> {
            &self.bound
        }
    }

    #[test]
    fn no_intersection_for_miss() {
        let left = coll(1, 8., 8., 10., 11.);

        let right = coll(2, 12., 13., 18., 18.);

        let mut sweep = SweepAndPrune2::new();
        let potentials = sweep.find_collider_pairs(&mut vec![left, right]);
        assert_eq!(0, potentials.len());
    }

    #[test]
    fn no_intersection_for_miss_unsorted() {
        let left = coll(1, 8., 8., 10., 11.);

        let right = coll(2, 12., 13., 18., 18.);

        let mut sweep = SweepAndPrune2::new();
        let potentials = sweep.find_collider_pairs(&mut vec![right, left]);
        assert_eq!(0, potentials.len());
    }

    #[test]
    fn intersection_for_hit() {
        let left = coll(1, 8., 8., 10., 11.);

        let right = coll(2, 9., 10., 18., 18.);

        let mut sweep = SweepAndPrune2::new();
        let potentials = sweep.find_collider_pairs(&mut vec![left.clone(), right.clone()]);
        assert_eq!(1, potentials.len());
        assert_eq!((0, 1), potentials[0]);
    }

    #[test]
    fn intersection_for_hit_unsorted() {
        let left = coll(1, 8., 8., 10., 11.);

        let right = coll(222, 9., 10., 18., 18.);

        let mut sweep = SweepAndPrune2::new();
        let potentials = sweep.find_collider_pairs(&mut vec![right.clone(), left.clone()]);
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
