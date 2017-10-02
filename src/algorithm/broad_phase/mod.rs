//! Broad phase collision detection algorithms

pub use self::brute_force::BruteForce;
pub use self::sweep_prune::{SweepAndPrune, SweepAndPrune2, SweepAndPrune3, Variance};

use dbvt::TreeValue;

mod brute_force;
mod sweep_prune;

/// Bound trait used by broad phase algorithms to borrow the bounding volume
pub trait HasBound {
    /// Bounding volume type
    type Bound;

    /// Borrow the bounding volume
    fn get_bound(&self) -> &Self::Bound;
}

impl<T> HasBound for (usize, T)
where
    T: TreeValue,
{
    type Bound = T::Bound;

    fn get_bound(&self) -> &Self::Bound {
        self.1.bound()
    }
}
