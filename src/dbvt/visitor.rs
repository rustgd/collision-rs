//! Concrete visitor implementations for use with
//! [`query`](struct.DynamicBoundingVolumeTree.html#method.query).
//!

use std::marker::PhantomData;

use cgmath::BaseFloat;

use {Bound, Frustum, Relation};
use prelude::*;
use super::{Visitor, TreeValue};

/// Visitor for doing continuous intersection testing on the DBVT.
///
/// Will return the result from the
/// [`Continuous`](../trait.Continuous.html) implementation
/// of bound.intersection(self.bound).
///
#[derive(Debug)]
pub struct ContinuousVisitor<'a, B: 'a, T> {
    bound: &'a B,
    marker: PhantomData<T>,
}

impl<'a, B: 'a, T> ContinuousVisitor<'a, B, T>
where
    T: TreeValue,
    T::Bound: Continuous<B> + Discrete<B>,
{
    /// Create a new visitor that will do continuous intersection tests using the given bound.
    ///
    pub fn new(bound: &'a B) -> Self {
        Self {
            bound,
            marker: PhantomData,
        }
    }
}

impl<'a, B: 'a, T> Visitor for ContinuousVisitor<'a, B, T>
where
    T: TreeValue,
    T::Bound: Continuous<B> + Discrete<B>,
{
    type Bound = T::Bound;
    type Result = <T::Bound as Continuous<B>>::Result;

    fn accept(&self, bound: &Self::Bound) -> Option<Self::Result> {
        bound.intersection(self.bound)
    }
}

/// Visitor for doing discrete intersection testing on the DBVT.
///
/// Will return () for intersections with the
/// [`Discrete`](../trait.Discrete.html) implementation
/// of bound.intersects(self.bound).
///
#[derive(Debug)]
pub struct DiscreteVisitor<'a, B: 'a, T> {
    bound: &'a B,
    marker: PhantomData<T>,
}

impl<'a, B: 'a, T> DiscreteVisitor<'a, B, T>
where
    T: TreeValue,
    T::Bound: Discrete<B>,
{
    /// Create a new visitor that will do discrete intersection tests using the given bound.
    pub fn new(bound: &'a B) -> Self {
        Self {
            bound,
            marker: PhantomData,
        }
    }
}

impl<'a, B: 'a, T> Visitor for DiscreteVisitor<'a, B, T>
where
    T: TreeValue,
    T::Bound: Discrete<B>,
{
    type Bound = T::Bound;
    type Result = ();

    fn accept(&self, bound: &Self::Bound) -> Option<()> {
        if bound.intersects(self.bound) {
            Some(())
        } else {
            None
        }
    }
}

/// Visitor for doing frustum intersection testing on the DBVT.
///
/// Will return the relation for intersections with the
/// [`Bound`](../trait.Bound.html) implementation
/// of self.frustum.contains(bound).
///
#[derive(Debug)]
pub struct FrustumVisitor<'a, S: 'a, T>
where
    S: BaseFloat,
{
    frustum: &'a Frustum<S>,
    marker: PhantomData<T>,
}

impl<'a, S, T> FrustumVisitor<'a, S, T>
where
    S: BaseFloat,
    T: TreeValue,
    T::Bound: Bound<S>,
{
    /// Create a new visitor that will do containment tests using the given frustum
    pub fn new(frustum: &'a Frustum<S>) -> Self {
        Self {
            frustum,
            marker: PhantomData,
        }
    }
}

impl<'a, S, T> Visitor for FrustumVisitor<'a, S, T>
where
    S: BaseFloat,
    T: TreeValue,
    T::Bound: Bound<S>,
{
    type Bound = T::Bound;
    type Result = Relation;

    fn accept(&self, bound: &Self::Bound) -> Option<Relation> {
        let r = self.frustum.contains(bound);
        if r == Relation::Out { None } else { Some(r) }
    }
}

#[cfg(test)]
mod tests {
    use cgmath::{Point2, Vector2, InnerSpace, PerspectiveFov, Deg, Point3, Vector3};

    use {Aabb, Aabb2, Ray2, Frustum, Projection, Aabb3, Relation};
    use super::*;
    use super::super::*;

    #[derive(Debug, Clone)]
    struct Value {
        index: usize,
        pub id: u32,
        pub aabb: Aabb2<f32>,
        fat_aabb: Aabb2<f32>,
    }

    impl Value {
        pub fn new(id: u32, aabb: Aabb2<f32>) -> Self {
            Self {
                index: 0,
                id,
                fat_aabb: aabb.add_margin(Vector2::new(3., 3.)),
                aabb,
            }
        }
    }

    impl TreeValue for Value {
        type Vector = Vector2<f32>;
        type Bound = Aabb2<f32>;

        fn bound(&self) -> &Aabb2<f32> {
            &self.aabb
        }

        fn fat_bound(&self) -> Aabb2<f32> {
            self.fat_aabb.clone()
        }

        fn set_index(&mut self, index: usize) {
            self.index = index
        }

        fn index(&self) -> usize {
            self.index
        }
    }

    #[derive(Debug, Clone)]
    struct Value3 {
        index: usize,
        pub id: u32,
        pub aabb: Aabb3<f32>,
        fat_aabb: Aabb3<f32>,
    }

    impl Value3 {
        pub fn new(id: u32, aabb: Aabb3<f32>) -> Self {
            Self {
                index: 0,
                id,
                fat_aabb: aabb.add_margin(Vector3::new(3., 3., 3.)),
                aabb,
            }
        }
    }

    impl TreeValue for Value3 {
        type Vector = Vector3<f32>;
        type Bound = Aabb3<f32>;

        fn bound(&self) -> &Aabb3<f32> {
            &self.aabb
        }

        fn fat_bound(&self) -> Aabb3<f32> {
            self.fat_aabb.clone()
        }

        fn set_index(&mut self, index: usize) {
            self.index = index
        }

        fn index(&self) -> usize {
            self.index
        }
    }

    #[test]
    fn test_ray_discrete() {
        let mut tree = DynamicBoundingVolumeTree::<Value>::new();
        tree.insert(Value::new(10, aabb2(5., 5., 10., 10.)));
        tree.insert(Value::new(11, aabb2(21., 14., 23., 16.)));
        tree.do_refit();

        let ray = Ray2::new(Point2::new(0., 0.), Vector2::new(-1., -1.).normalize());
        let visitor = DiscreteVisitor::<Ray2<f32>, Value>::new(&ray);
        assert_eq!(0, tree.query(&visitor).len());

        let ray = Ray2::new(Point2::new(6., 0.), Vector2::new(0., 1.).normalize());
        let visitor = DiscreteVisitor::<Ray2<f32>, Value>::new(&ray);
        let results = tree.query(&visitor);
        assert_eq!(1, results.len());
        assert_eq!(10, results[0].0.id);
    }

    #[test]
    fn test_ray_continuous() {
        let mut tree = DynamicBoundingVolumeTree::<Value>::new();
        tree.insert(Value::new(10, aabb2(5., 5., 10., 10.)));
        tree.insert(Value::new(11, aabb2(21., 14., 23., 16.)));
        tree.do_refit();

        let ray = Ray2::new(Point2::new(0., 0.), Vector2::new(-1., -1.).normalize());
        let visitor = ContinuousVisitor::<Ray2<f32>, Value>::new(&ray);
        assert_eq!(0, tree.query(&visitor).len());

        let ray = Ray2::new(Point2::new(6., 0.), Vector2::new(0., 1.).normalize());
        let visitor = ContinuousVisitor::<Ray2<f32>, Value>::new(&ray);
        let results = tree.query(&visitor);
        assert_eq!(1, results.len());
        assert_eq!(10, results[0].0.id);
        assert_eq!(Point2::new(6., 5.), results[0].1);
    }

    #[test]
    fn test_frustum() {
        let mut tree = DynamicBoundingVolumeTree::<Value3>::new();
        tree.insert(Value3::new(10, aabb3(0.2, 0.2, 0.2, 10., 10., 10.)));
        tree.insert(Value3::new(11, aabb3(0.2, 0.2, -0.2, 10., 10., -10.)));
        tree.do_refit();

        let frustum = frustum();
        let visitor = FrustumVisitor::<f32, Value3>::new(&frustum);
        let result = tree.query(&visitor);
        assert_eq!(1, result.len());
        let (v, r) = result[0];
        assert_eq!(Relation::Cross, r);
        assert_eq!(11, v.id);
    }

    fn aabb2(minx: f32, miny: f32, maxx: f32, maxy: f32) -> Aabb2<f32> {
        Aabb2::new(Point2::new(minx, miny), Point2::new(maxx, maxy))
    }

    fn aabb3(minx: f32, miny: f32, minz: f32, maxx: f32, maxy: f32, maxz: f32) -> Aabb3<f32> {
        Aabb3::new(Point3::new(minx, miny, minz), Point3::new(maxx, maxy, maxz))
    }

    // Default perspective projection is looking down the negative z axis
    fn frustum() -> Frustum<f32> {
        let projection = PerspectiveFov {
            fovy : Deg(60.).into(),
            aspect : 16. / 9.,
            near : 0.1,
            far : 4.0,
        };
        projection.to_frustum()
    }
}