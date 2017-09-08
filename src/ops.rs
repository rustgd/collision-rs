
pub trait Union<RHS = Self> {
    type Output;

    fn union(&self, &RHS) -> Self::Output;
}
