use cgmath::prelude::*;
use cgmath::{Deg, PerspectiveFov, Point2, Point3, Vector2, Vector3};
use collision::dbvt::*;
use collision::prelude::*;
use collision::{Aabb2, Aabb3, Frustum, Projection, Ray2, Relation};
use rand::Rng;

#[derive(Debug, Clone)]
struct Value2 {
    pub id: u32,
    pub aabb: Aabb2<f32>,
    fat_aabb: Aabb2<f32>,
}

impl Value2 {
    pub fn new(id: u32, aabb: Aabb2<f32>) -> Self {
        Self {
            id,
            fat_aabb: aabb.add_margin(Vector2::new(0., 0.)),
            aabb,
        }
    }
}

impl TreeValue for Value2 {
    type Bound = Aabb2<f32>;

    fn bound(&self) -> &Aabb2<f32> {
        &self.aabb
    }

    fn get_bound_with_margin(&self) -> Aabb2<f32> {
        self.fat_aabb
    }
}

#[derive(Debug, Clone)]
struct Value3 {
    pub id: u32,
    pub aabb: Aabb3<f32>,
    fat_aabb: Aabb3<f32>,
}

impl Value3 {
    pub fn new(id: u32, aabb: Aabb3<f32>) -> Self {
        Self {
            id,
            fat_aabb: aabb.add_margin(Vector3::new(3., 3., 3.)),
            aabb,
        }
    }
}

impl TreeValue for Value3 {
    type Bound = Aabb3<f32>;

    fn bound(&self) -> &Aabb3<f32> {
        &self.aabb
    }

    fn get_bound_with_margin(&self) -> Aabb3<f32> {
        self.fat_aabb
    }
}

#[test]
fn test_add_1() {
    let mut tree = DynamicBoundingVolumeTree::<Value2>::new();
    tree.insert(Value2::new(10, aabb2(5., 5., 10., 10.)));
    assert_eq!(1, tree.values().len());
    assert_eq!(1, tree.size());
    assert_eq!(1, tree.height());
}

#[test]
fn test_add_2() {
    let mut tree = DynamicBoundingVolumeTree::<Value2>::new();
    tree.insert(Value2::new(10, aabb2(5., 5., 10., 10.)));
    tree.insert(Value2::new(11, aabb2(21., 14., 23., 16.)));
    tree.do_refit();
    assert_eq!(2, tree.values().len());
    assert_eq!(3, tree.size());
    assert_eq!(2, tree.height());
}

#[test]
fn test_add_3() {
    let mut tree = DynamicBoundingVolumeTree::<Value2>::new();
    tree.insert(Value2::new(10, aabb2(5., 5., 10., 10.)));
    tree.insert(Value2::new(11, aabb2(21., 14., 23., 16.)));
    tree.insert(Value2::new(12, aabb2(-12., -1., 0., 0.)));
    tree.do_refit();
    assert_eq!(3, tree.values().len());
    assert_eq!(5, tree.size());
    assert_eq!(3, tree.height());
}

#[test]
fn test_add_5() {
    let mut tree = DynamicBoundingVolumeTree::<Value2>::new();
    tree.insert(Value2::new(10, aabb2(5., 5., 10., 10.)));
    tree.do_refit();
    tree.insert(Value2::new(11, aabb2(21., 14., 23., 16.)));
    tree.do_refit();
    tree.insert(Value2::new(12, aabb2(-12., -1., 0., 0.)));
    tree.do_refit();
    tree.insert(Value2::new(13, aabb2(-200., -26., -185., -20.)));
    tree.do_refit();
    tree.insert(Value2::new(14, aabb2(56., 58., 99., 96.)));
    tree.do_refit();
    assert_eq!(5, tree.values().len());
    assert_eq!(9, tree.size());
    assert_eq!(4, tree.height());
}

#[test]
fn test_add_20() {
    let mut tree = DynamicBoundingVolumeTree::<Value2>::new();
    let mut rng = rand::thread_rng();
    for i in 0..20 {
        let offset = rng.gen_range(-10.0..10.0);
        tree.insert(Value2::new(
            i,
            aabb2(offset + 0.1, offset + 0.1, offset + 0.3, offset + 0.3),
        ));
        tree.do_refit();
    }
    assert_eq!(20, tree.values().len());
    assert_eq!(39, tree.size());
}

#[test]
fn test_remove_leaf_right_side() {
    let mut tree = DynamicBoundingVolumeTree::<Value2>::new();
    tree.insert(Value2::new(10, aabb2(5., 5., 10., 10.)));
    tree.insert(Value2::new(11, aabb2(21., 14., 23., 16.)));
    tree.insert(Value2::new(12, aabb2(-12., -1., 0., 0.)));
    tree.insert(Value2::new(13, aabb2(-200., -26., -185., -20.)));
    tree.insert(Value2::new(14, aabb2(56., 58., 99., 96.)));
    tree.do_refit();
    let node_index = tree.values()[4].0;
    tree.remove(node_index);
    tree.do_refit();
    assert_eq!(4, tree.values().len());
    assert_eq!(7, tree.size());
    assert_eq!(4, tree.height());
}

#[test]
fn test_remove_leaf_left_side() {
    let mut tree = DynamicBoundingVolumeTree::<Value2>::new();
    tree.insert(Value2::new(10, aabb2(5., 5., 10., 10.)));
    tree.insert(Value2::new(11, aabb2(21., 14., 23., 16.)));
    tree.insert(Value2::new(12, aabb2(-12., -1., 0., 0.)));
    tree.insert(Value2::new(13, aabb2(-200., -26., -185., -20.)));
    tree.insert(Value2::new(14, aabb2(56., 58., 99., 96.)));
    tree.do_refit();
    let node_index = tree.values()[3].0;
    tree.remove(node_index);
    tree.do_refit();
    assert_eq!(4, tree.values().len());
    assert_eq!(7, tree.size());
    assert_eq!(3, tree.height());
}

#[test]
fn test_remove_leaf_left_side_2() {
    let mut tree = DynamicBoundingVolumeTree::<Value2>::new();
    tree.insert(Value2::new(10, aabb2(5., 5., 10., 10.)));
    tree.insert(Value2::new(11, aabb2(21., 14., 23., 16.)));
    tree.insert(Value2::new(12, aabb2(-12., -1., 0., 0.)));
    tree.insert(Value2::new(13, aabb2(-200., -26., -185., -20.)));
    tree.insert(Value2::new(14, aabb2(56., 58., 99., 96.)));
    tree.do_refit();
    let node_index = tree.values()[3].0;
    tree.remove(node_index);
    let node_index = tree.values()[0].0;
    tree.remove(node_index);
    tree.do_refit();
    assert_eq!(3, tree.values().len());
    assert_eq!(5, tree.size());
    assert_eq!(3, tree.height());
}

#[test]
fn test_remove_leaf_left_side_3() {
    let mut tree = DynamicBoundingVolumeTree::<Value2>::new();
    tree.insert(Value2::new(10, aabb2(5., 5., 10., 10.)));
    tree.insert(Value2::new(11, aabb2(21., 14., 23., 16.)));
    tree.insert(Value2::new(12, aabb2(-12., -1., 0., 0.)));
    tree.insert(Value2::new(13, aabb2(-200., -26., -185., -20.)));
    tree.insert(Value2::new(14, aabb2(56., 58., 99., 96.)));
    tree.do_refit();
    let node_index = tree.values()[3].0;
    tree.remove(node_index);
    let node_index = tree.values()[0].0;
    tree.remove(node_index);
    let node_index = tree.values()[1].0;
    tree.remove(node_index);
    tree.do_refit();
    assert_eq!(2, tree.values().len());
    assert_eq!(3, tree.size());
    assert_eq!(2, tree.height());
}

#[test]
fn test_remove_leaf_left_right() {
    let mut tree = DynamicBoundingVolumeTree::<Value2>::new();
    tree.insert(Value2::new(10, aabb2(5., 5., 10., 10.)));
    tree.insert(Value2::new(11, aabb2(21., 14., 23., 16.)));
    tree.insert(Value2::new(12, aabb2(-12., -1., 0., 0.)));
    tree.insert(Value2::new(13, aabb2(-200., -26., -185., -20.)));
    tree.insert(Value2::new(14, aabb2(56., 58., 99., 96.)));
    tree.do_refit();
    let node_index = tree.values()[3].0;
    tree.remove(node_index);
    let node_index = tree.values()[0].0;
    tree.remove(node_index);
    let node_index = tree.values()[0].0;
    tree.remove(node_index);
    tree.do_refit();
    assert_eq!(2, tree.values().len());
    assert_eq!(3, tree.size());
    assert_eq!(2, tree.height());
}

#[test]
fn test_remove_second_to_last() {
    let mut tree = DynamicBoundingVolumeTree::<Value2>::new();
    tree.insert(Value2::new(10, aabb2(5., 5., 10., 10.)));
    tree.insert(Value2::new(11, aabb2(21., 14., 23., 16.)));
    tree.insert(Value2::new(12, aabb2(-12., -1., 0., 0.)));
    tree.insert(Value2::new(13, aabb2(-200., -26., -185., -20.)));
    tree.insert(Value2::new(14, aabb2(56., 58., 99., 96.)));
    tree.do_refit();
    let node_index = tree.values()[3].0;
    tree.remove(node_index);
    let node_index = tree.values()[0].0;
    tree.remove(node_index);
    let node_index = tree.values()[0].0;
    tree.remove(node_index);
    let node_index = tree.values()[0].0;
    tree.remove(node_index);
    tree.do_refit();
    assert_eq!(1, tree.values().len());
    assert_eq!(1, tree.size());
    assert_eq!(1, tree.height());
}

#[test]
fn test_remove_last() {
    let mut tree = DynamicBoundingVolumeTree::<Value2>::new();
    tree.insert(Value2::new(10, aabb2(5., 5., 10., 10.)));
    tree.insert(Value2::new(11, aabb2(21., 14., 23., 16.)));
    tree.insert(Value2::new(12, aabb2(-12., -1., 0., 0.)));
    tree.insert(Value2::new(13, aabb2(-200., -26., -185., -20.)));
    tree.insert(Value2::new(14, aabb2(56., 58., 99., 96.)));
    tree.do_refit();
    let node_index = tree.values()[3].0;
    tree.remove(node_index);
    let node_index = tree.values()[0].0;
    tree.remove(node_index);
    let node_index = tree.values()[0].0;
    tree.remove(node_index);
    let node_index = tree.values()[0].0;
    tree.remove(node_index);
    let node_index = tree.values()[0].0;
    tree.remove(node_index);
    tree.do_refit();
    assert_eq!(0, tree.values().len());
    assert_eq!(0, tree.size());
    assert_eq!(0, tree.height());
}

#[test]
fn test_ray_closest() {
    let mut tree = DynamicBoundingVolumeTree::<Value2>::new();
    tree.insert(Value2::new(12, aabb2(0., 0., 10., 10.)));
    tree.insert(Value2::new(33, aabb2(15., 3., 3., 3.)));
    tree.insert(Value2::new(13, aabb2(-145., 34., 2., 2.)));
    tree.insert(Value2::new(66, aabb2(123., -10., 10., 10.)));
    tree.insert(Value2::new(1, aabb2(22., 50., 16., 16.)));
    tree.insert(Value2::new(76, aabb2(7., 3., 1., 1.)));
    tree.insert(Value2::new(99, aabb2(19., -12., 3., 3.)));
    tree.do_refit();

    let result = query_ray_closest(
        &tree,
        Ray2::new(Point2::new(12., 12.), Vector2::new(0.5, -0.5).normalize()),
    );
    assert!(result.is_some());
    let (v, p) = result.unwrap();
    assert_eq!(33, v.id);
    assert_eq!(Point2::new(18., 5.9999995), p);
}

#[test]
fn test_ray_discrete() {
    let mut tree = DynamicBoundingVolumeTree::<Value2>::new();
    tree.insert(Value2::new(10, aabb2(5., 5., 5., 15.)));
    tree.insert(Value2::new(11, aabb2(21., 14., 2., 2.)));
    tree.do_refit();

    let ray = Ray2::new(Point2::new(0., 0.), Vector2::new(-1., -1.).normalize());
    let mut visitor = DiscreteVisitor::<Ray2<f32>, Value2>::new(&ray);
    assert_eq!(0, tree.query(&mut visitor).len());

    let ray = Ray2::new(Point2::new(6., 0.), Vector2::new(0., 1.).normalize());
    let mut visitor = DiscreteVisitor::<Ray2<f32>, Value2>::new(&ray);
    let results = tree.query(&mut visitor);
    assert_eq!(1, results.len());
    assert_eq!(10, results[0].0.id);
}

#[test]
fn test_ray_continuous() {
    let mut tree = DynamicBoundingVolumeTree::<Value2>::new();
    tree.insert(Value2::new(10, aabb2(5., 5., 5., 15.)));
    tree.insert(Value2::new(11, aabb2(21., 14., 2., 2.)));
    tree.do_refit();

    let ray = Ray2::new(Point2::new(0., 0.), Vector2::new(-1., -1.).normalize());
    let mut visitor = ContinuousVisitor::<Ray2<f32>, Value2>::new(&ray);
    assert_eq!(0, tree.query(&mut visitor).len());

    let ray = Ray2::new(Point2::new(6., 0.), Vector2::new(0., 1.).normalize());
    let mut visitor = ContinuousVisitor::<Ray2<f32>, Value2>::new(&ray);
    let results = tree.query(&mut visitor);
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
    let mut visitor = FrustumVisitor::<f32, Value3>::new(&frustum);
    let result = tree.query(&mut visitor);
    assert_eq!(1, result.len());
    let (v, r) = result[0];
    assert_eq!(Relation::Cross, r);
    assert_eq!(11, v.id);
}

fn aabb2(minx: f32, miny: f32, width: f32, height: f32) -> Aabb2<f32> {
    Aabb2::new(
        Point2::new(minx, miny),
        Point2::new(minx + width, miny + height),
    )
}

fn aabb3(minx: f32, miny: f32, minz: f32, maxx: f32, maxy: f32, maxz: f32) -> Aabb3<f32> {
    Aabb3::new(Point3::new(minx, miny, minz), Point3::new(maxx, maxy, maxz))
}

// Default perspective projection is looking down the negative z axis
fn frustum() -> Frustum<f32> {
    let projection = PerspectiveFov {
        fovy: Deg(60.).into(),
        aspect: 16. / 9.,
        near: 0.1,
        far: 4.0,
    };
    projection.to_frustum()
}
