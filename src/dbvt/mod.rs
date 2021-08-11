//! A [dynamic bounding volume tree implementation](struct.DynamicBoundingVolumeTree.html),
//! index based (not pointer based).
//!
//! The following invariants are true:
//!
//! * A branch node must have exactly two children.
//! * Only leaf nodes contain user data.
//!
//! Internal nodes may have incorrect bounding volumes and height after insertion, removal and
//! updates to values in the tree. These will be fixed during refitting, which is done by calling
//! [`do_refit`](struct.DynamicBoundingVolumeTree.html#method.do_refit).
//!
//! The main heuristic used for insertion and tree rotation, is surface area of the bounding volume.
//!
//! Updating of values in the tree, can either be performed by using the
//! [`values`](struct.DynamicBoundingVolumeTree.html#method.values) function to get a mutable
//! iterator over the values in the tree, or by using
//! [`update_node`](struct.DynamicBoundingVolumeTree.html#method.update_node).
//! It is recommended to use the latter when possible. If the former is used,
//! [`reindex_values`](struct.DynamicBoundingVolumeTree.html#method.reindex_values)
//! must be called if the order of the values is changed in any way.
//!
//! The trait [`TreeValue`](trait.TreeValue.html) needs to be implemented for a type to be usable
//! in the tree.
//!
//! # Examples
//!
//! ```
//! # extern crate cgmath;
//! # extern crate collision;
//!
//! use cgmath::{Point2, Vector2, InnerSpace};
//! use collision::{Aabb, Aabb2, Ray2};
//!
//! use collision::dbvt::{DynamicBoundingVolumeTree, TreeValue, ContinuousVisitor};
//!
//! #[derive(Debug, Clone)]
//! struct Value {
//!     pub id: u32,
//!     pub aabb: Aabb2<f32>,
//!     fat_aabb: Aabb2<f32>,
//! }
//!
//! impl Value {
//!     pub fn new(id: u32, aabb: Aabb2<f32>) -> Self {
//!         Self {
//!             id,
//!             fat_aabb : aabb.add_margin(Vector2::new(3., 3.)),
//!             aabb,
//!         }
//!     }
//! }
//!
//! impl TreeValue for Value {
//!     type Bound = Aabb2<f32>;
//!
//!     fn bound(&self) -> &Aabb2<f32> {
//!         &self.aabb
//!     }
//!
//!     fn get_bound_with_margin(&self) -> Aabb2<f32> {
//!         self.fat_aabb.clone()
//!     }
//! }
//!
//! fn aabb2(minx: f32, miny: f32, maxx: f32, maxy: f32) -> Aabb2<f32> {
//!    Aabb2::new(Point2::new(minx, miny), Point2::new(maxx, maxy))
//! }
//!
//! fn main() {
//!     let mut tree = DynamicBoundingVolumeTree::<Value>::new();
//!     tree.insert(Value::new(10, aabb2(5., 5., 10., 10.)));
//!     tree.insert(Value::new(11, aabb2(21., 14., 23., 16.)));
//!     tree.do_refit();
//!
//!     let ray = Ray2::new(Point2::new(0., 0.), Vector2::new(-1., -1.).normalize());
//!     let mut visitor = ContinuousVisitor::<Ray2<f32>, Value>::new(&ray);
//!     assert_eq!(0, tree.query(&mut visitor).len());
//!
//!     let ray = Ray2::new(Point2::new(6., 0.), Vector2::new(0., 1.).normalize());
//!     let mut visitor = ContinuousVisitor::<Ray2<f32>, Value>::new(&ray);
//!     let results = tree.query(&mut visitor);
//!     assert_eq!(1, results.len());
//!     assert_eq!(10, results[0].0.id);
//!     assert_eq!(Point2::new(6., 5.), results[0].1);
//! }
//! ```
//!

pub use self::util::*;
pub use self::visitor::*;
pub use self::wrapped::TreeValueWrapped;

use std::cmp::max;
use std::fmt;

use cgmath::num_traits::NumCast;
use rand;
use rand::Rng;

use crate::prelude::*;

mod util;
mod visitor;
mod wrapped;

const SURFACE_AREA_IMPROVEMENT_FOR_ROTATION: f32 = 0.3;
const PERFORM_ROTATION_PERCENTAGE: u32 = 10;

/// Trait that needs to be implemented for any value that is to be used in the
/// [`DynamicBoundingVolumeTree`](struct.DynamicBoundingVolumeTree.html).
///
pub trait TreeValue: Clone {
    /// Bounding volume type
    type Bound;

    /// Return the bounding volume of the value
    fn bound(&self) -> &Self::Bound;

    /// Return a fattened bounding volume. For shapes that do not move, this can be the same as the
    /// base bounding volume. It is recommended for moving shapes to have a larger fat bound, so
    /// tree rotations don't have to be performed every frame.
    fn get_bound_with_margin(&self) -> Self::Bound;
}

/// Make it possible to run broad phase algorithms directly on the value storage in DBVT
impl<T> HasBound for (usize, T)
where
    T: TreeValue,
    T::Bound: Bound,
{
    type Bound = T::Bound;

    fn bound(&self) -> &Self::Bound {
        self.1.bound()
    }
}

/// Visitor trait used for [querying](struct.DynamicBoundingVolumeTree.html#method.query) the tree.
pub trait Visitor {
    /// Bounding volume accepted by the visitor
    type Bound;

    /// Result returned by the acceptance test
    type Result;

    /// Acceptance test function
    fn accept(&mut self, bound: &Self::Bound, is_leaf: bool) -> Option<Self::Result>;
}

/// A dynamic bounding volume tree, index based (not pointer based).
///
/// The following invariants are true:
///
/// * A branch node must have exactly two children.
/// * Only leaf nodes contain user data.
///
/// Internal nodes may have incorrect bounding volumes and height after insertion, removal and
/// updates to values in the tree. These will be fixed during refitting, which is done by calling
/// [`do_refit`](struct.DynamicBoundingVolumeTree.html#method.do_refit). This function should
/// ideally not be called more than once per frame.
///
/// The main heuristic used for insertion and tree rotation, is surface area of the bounding volume.
///
/// Updating of values in the tree, can either be performed by using the
/// [`values`](struct.DynamicBoundingVolumeTree.html#method.values) function to get a mutable
/// iterator over the values in the tree, or by using
/// [`update_node`](struct.DynamicBoundingVolumeTree.html#method.update_node).
/// It is recommended to use the latter when possible. If the former is used,
/// [`reindex_values`](struct.DynamicBoundingVolumeTree.html#method.reindex_values)
/// must be called if the order of the values is changed in any way.
///
/// # Type parameters:
///
/// - `T`: A type that implements [`TreeValue`](trait.TreeValue.html), and is usable in the tree.
///        Needs to be able to store the node index of itself, and handle its own bound and
///        fattened bound.
///
/// # Examples
///
/// ```
/// # extern crate cgmath;
/// # extern crate collision;
///
/// use cgmath::{Point2, Vector2, InnerSpace};
/// use collision::{Aabb, Aabb2, Ray2};
/// use collision::dbvt::{DynamicBoundingVolumeTree, TreeValue, ContinuousVisitor};
///
/// #[derive(Debug, Clone)]
/// struct Value {
///     pub id: u32,
///     pub aabb: Aabb2<f32>,
///     fat_aabb: Aabb2<f32>,
/// }
///
/// impl Value {
///     pub fn new(id: u32, aabb: Aabb2<f32>) -> Self {
///         Self {
///             id,
///             fat_aabb : aabb.add_margin(Vector2::new(3., 3.)),
///             aabb,
///         }
///     }
/// }
///
/// impl TreeValue for Value {
///     type Bound = Aabb2<f32>;
///
///     fn bound(&self) -> &Aabb2<f32> {
///         &self.aabb
///     }
///
///     fn get_bound_with_margin(&self) -> Aabb2<f32> {
///         self.fat_aabb.clone()
///     }
/// }
///
/// fn aabb2(minx: f32, miny: f32, maxx: f32, maxy: f32) -> Aabb2<f32> {
///    Aabb2::new(Point2::new(minx, miny), Point2::new(maxx, maxy))
/// }
///
/// fn main() {
///     let mut tree = DynamicBoundingVolumeTree::<Value>::new();
///     tree.insert(Value::new(10, aabb2(5., 5., 10., 10.)));
///     tree.insert(Value::new(11, aabb2(21., 14., 23., 16.)));
///     tree.do_refit();
///
///     let ray = Ray2::new(Point2::new(0., 0.), Vector2::new(-1., -1.).normalize());
///     let mut visitor = ContinuousVisitor::<Ray2<f32>, Value>::new(&ray);
///     assert_eq!(0, tree.query(&mut visitor).len());
///
///     let ray = Ray2::new(Point2::new(6., 0.), Vector2::new(0., 1.).normalize());
///     let mut visitor = ContinuousVisitor::<Ray2<f32>, Value>::new(&ray);
///     let results = tree.query(&mut visitor);
///     assert_eq!(1, results.len());
///     assert_eq!(10, results[0].0.id);
///     assert_eq!(Point2::new(6., 5.), results[0].1);
/// }
/// ```
///
pub struct DynamicBoundingVolumeTree<T>
where
    T: TreeValue,
{
    nodes: Vec<Node<T::Bound>>,
    values: Vec<(usize, T)>,
    free_list: Vec<usize>,
    updated_list: Vec<usize>,
    root_index: usize,
    refit_nodes: Vec<(u32, usize)>,
}

impl<T> Default for DynamicBoundingVolumeTree<T>
where
    T: TreeValue,
{
    fn default() -> Self {
        DynamicBoundingVolumeTree {
            // we add Nil to first position so only the root node can have parent = 0
            nodes: vec![Node::Nil],
            values: Vec::default(),
            free_list: Vec::default(),
            updated_list: Vec::default(),
            root_index: 0,
            refit_nodes: Vec::default(),
        }
    }
}

impl<T> fmt::Debug for DynamicBoundingVolumeTree<T>
where
    T: TreeValue,
    T::Bound: fmt::Debug,
{
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "graph tree {{")?;
        for n_index in 1..self.nodes.len() {
            match self.nodes[n_index] {
                Node::Branch(ref b) => {
                    write!(f, "  n_{} [label=\"{:?}\"];", n_index, b.bound)?;
                    write!(f, "  n_{} -- n_{};", n_index, b.left)?;
                    write!(f, "  n_{} -- n_{};", n_index, b.right)?;
                }

                Node::Leaf(ref l) => {
                    write!(f, "  n_{} [label=\"{:?}\"];", n_index, l.bound)?;
                }

                Node::Nil => (),
            }
        }
        write!(f, "}}")
    }
}

/// Branch node
#[derive(Debug)]
struct Branch<B> {
    parent: usize,
    left: usize,
    right: usize,
    height: u32,
    bound: B,
}

/// Leaf node
#[derive(Debug)]
struct Leaf<B> {
    parent: usize,
    value: usize,
    bound: B,
}

/// Nodes
#[derive(Debug)]
enum Node<B> {
    Branch(Branch<B>),
    Leaf(Leaf<B>),
    Nil,
}

impl<T> DynamicBoundingVolumeTree<T>
where
    T: TreeValue,
    T::Bound: Clone + Contains<T::Bound> + Union<T::Bound, Output = T::Bound> + SurfaceArea,
{
    /// Create a new tree.
    ///
    /// ### Type parameters:
    ///
    /// - `T`: A type that implements [`TreeValue`](trait.TreeValue.html), and is usable in the
    ///        tree. Needs to be able to store the node index of itself, and handle its own bound
    ///        and fattened bound.
    /// - `T::Bound`: Bounding volume type that implements the following collision-rs traits:
    ///              [`Contains`][1] on itself, [`Union`][2] on itself, and [`SurfaceArea`][3].
    ///
    /// [1]: ../trait.Contains.html
    /// [2]: ../trait.Union.html
    /// [3]: ../trait.SurfaceArea.html
    ///
    pub fn new() -> Self {
        Default::default()
    }

    /// Return the number of nodes in the tree.
    ///
    pub fn size(&self) -> usize {
        // -1 because the first slot in the nodes vec is never used
        self.nodes.len() - self.free_list.len() - 1
    }

    /// Return the height of the root node. Leafs are considered to have height 1.
    ///
    pub fn height(&self) -> u32 {
        if self.values.is_empty() {
            0
        } else {
            match self.nodes[self.root_index] {
                Node::Branch(ref b) => b.height,
                Node::Leaf(_) => 1,
                Node::Nil => 0,
            }
        }
    }

    /// Get an immutable list of all values in the tree.
    ///
    /// ### Returns
    ///
    /// A immutable reference to the [`Vec`](https://doc.rust-lang.org/std/vec/struct.Vec.html)
    /// of values in the tree.
    ///
    pub fn values(&self) -> &Vec<(usize, T)> {
        &self.values
    }

    /// Get a mutable list of all values in the tree.
    ///
    /// Do not insert or remove values directly in this list, instead use
    /// [`insert`](struct.DynamicBoundingVolumeTree.html#method.insert) and
    /// [`remove`](struct.DynamicBoundingVolumeTree.html#method.remove)
    /// on the tree. It is allowed to change the order of the values, but when doing so it is
    /// required to use
    /// [`reindex_values`](struct.DynamicBoundingVolumeTree.html#method.reindex_values)
    /// after changing the order, and before any other operation
    /// is performed on the tree. Otherwise the internal consistency of the tree will be broken.
    ///
    /// Do not change the first value in the tuple, this is the node index of the value, and without
    /// that the tree will not function.
    ///
    /// ### Returns
    ///
    /// A mutable reference to the [`Vec`](https://doc.rust-lang.org/std/vec/struct.Vec.html)
    /// of values in the tree.
    ///
    pub fn values_mut(&mut self) -> &mut Vec<(usize, T)> {
        &mut self.values
    }

    /// Reindex the values list, making sure that nodes in the tree point to the correct entry in
    /// the values list.
    ///
    /// Complexity is O(n).
    ///
    pub fn reindex_values(&mut self) {
        for i in 0..self.values.len() {
            if let Node::Leaf(ref mut leaf) = self.nodes[self.values[i].0] {
                leaf.value = i;
            }
        }
    }

    /// Clear the tree.
    ///
    /// Will remove all nodes and their values.
    ///
    pub fn clear(&mut self) {
        self.root_index = 0;
        self.nodes = vec![Node::Nil];
        self.free_list.clear();
        self.refit_nodes.clear();
        self.values.clear();
    }

    /// Return the value index for the given node index.
    pub fn value_index(&self, node_index: usize) -> Option<usize> {
        match self.nodes[node_index] {
            Node::Leaf(ref leaf) => Some(leaf.value),
            _ => None,
        }
    }

    /// Query the tree for all leafs that the given visitor accepts.
    ///
    /// Will do a depth first search of the tree and pass all bounding volumes on the way to the
    /// visitor.
    ///
    /// This function have approximate complexity O(log^2 n).
    ///
    /// ### Parameters:
    ///
    /// - `visitor`: The visitor to check for bounding volume tests.
    ///
    /// ### Type parameters:
    ///
    /// - `V`: Type that implements of [`Visitor`](trait.Visitor.html)
    ///
    /// ### Returns
    ///
    /// Will return a list of tuples of values accepted and the result returned by the visitor for
    /// the acceptance test.
    ///
    pub fn query<V>(&self, visitor: &mut V) -> Vec<(&T, V::Result)>
    where
        V: Visitor<Bound = T::Bound>,
    {
        self.query_for_indices(visitor)
            .into_iter()
            .map(|(value_index, result)| (&self.values[value_index].1, result))
            .collect()
    }

    /// Query the tree for all leafs that the given visitor accepts.
    ///
    /// Will do a depth first search of the tree and pass all bounding volumes on the way to the
    /// visitor.
    ///
    /// This function have approximate complexity O(log^2 n).
    ///
    /// ### Parameters:
    ///
    /// - `visitor`: The visitor to check for bounding volume tests.
    ///
    /// ### Type parameters:
    ///
    /// - `V`: Type that implements of [`Visitor`](trait.Visitor.html)
    ///
    /// ### Returns
    ///
    /// Will return a list of tuples of value indices accepted and the result returned by the
    /// visitor for the acceptance test.
    ///
    pub fn query_for_indices<V>(&self, visitor: &mut V) -> Vec<(usize, V::Result)>
    where
        V: Visitor<Bound = T::Bound>,
    {
        let mut stack = [0; 256];
        stack[0] = self.root_index;
        let mut stack_pointer = 1;
        let mut values = Vec::default();
        while stack_pointer > 0 {
            // depth search, use last added as next test subject
            stack_pointer -= 1;
            let node_index = stack[stack_pointer];
            let node = &self.nodes[node_index];

            match *node {
                Node::Leaf(ref leaf) => {
                    // if we encounter a leaf, do a real bound intersection test, and add to return
                    // values if there's an intersection
                    if let Some(result) = visitor.accept(self.values[leaf.value].1.bound(), true) {
                        values.push((leaf.value, result));
                    }
                }

                // if we encounter a branch, do intersection test, and push the children if the
                // branch intersected
                Node::Branch(ref branch) => {
                    if visitor.accept(&branch.bound, false).is_some() {
                        stack[stack_pointer] = branch.left;
                        stack[stack_pointer + 1] = branch.right;
                        stack_pointer += 2;
                    }
                }
                Node::Nil => (),
            }
        }
        values
    }

    /// Update a node in the tree with a new value.
    ///
    /// The node will be fed its node_index after updating in the tree, so there is no need to
    /// add that manually in the value.
    ///
    /// Will cause the node to be updated and be flagged as updated, which will cause
    /// [`update`](struct.DynamicBoundingVolumeTree.html#method.update) to process the node the next
    /// time it is called.
    ///
    /// ### Parameters
    ///
    /// - `node_index`: index of the node to update
    /// - `new_value`: the new value to write in that node
    ///
    pub fn update_node(&mut self, node_index: usize, new_value: T) {
        if let Node::Leaf(ref mut leaf) = self.nodes[node_index] {
            self.values[leaf.value].1 = new_value;
        }
        self.flag_updated(node_index);
    }

    /// Flag a node as having been updated (moved/rotated).
    ///
    /// Will cause [`update`](struct.DynamicBoundingVolumeTree.html#method.update) to process the
    /// node the next time it is called.
    ///
    /// ### Parameters
    ///
    /// - `node_index`: the node index of the updated node
    ///
    pub fn flag_updated(&mut self, node_index: usize) {
        self.updated_list.push(node_index);
    }

    /// Go through the updated list and check the fat bounds in the tree.
    ///
    /// After updating values in the values list, it is possible that some of the leafs values have
    /// outgrown their fat bounds. If so, they may need to be moved in the tree. This is done during
    /// refitting.
    ///
    /// Note that no parents have their bounds/height updated directly by this function, instead
    /// [`do_refit`](struct.DynamicBoundingVolumeTree.html#method.do_refit) should be called after
    /// all insert/remove/updates have been performed this frame.
    ///
    pub fn update(&mut self) {
        let nodes = self
            .updated_list
            .iter()
            .filter_map(|&index| {
                if let Node::Leaf(ref l) = self.nodes[index] {
                    if !l.bound.contains(self.values[l.value].1.bound()) {
                        Some((
                            index,
                            l.parent,
                            self.values[l.value].1.get_bound_with_margin(),
                        ))
                    } else {
                        None
                    }
                } else {
                    None
                }
            })
            .collect::<Vec<(usize, usize, T::Bound)>>();

        for (node_index, parent_index, fat_bound) in nodes {
            if let Node::Leaf(ref mut leaf) = self.nodes[node_index] {
                leaf.bound = fat_bound;
            }
            self.mark_for_refit(parent_index, 2);
        }

        self.updated_list.clear();
    }

    /// Utility method to perform updates and refitting. Should be called once per frame.
    ///
    /// Will in turn call [`update`](struct.DynamicBoundingVolumeTree.html#method.update), followed
    /// by [`do_refit`](struct.DynamicBoundingVolumeTree.html#method.do_refit).
    ///
    pub fn tick(&mut self) {
        self.update();
        self.do_refit();
    }

    /// Insert a value into the tree.
    ///
    /// This will search the tree for the best leaf to pair the value up with, using the surface
    /// area of the value's bounding volume as the main heuristic. Will always cause a new branch
    /// node and a new leaf node (containing the given value) to be added to the tree.
    /// This is to keep the invariant of branches always having 2 children true.
    ///
    /// This function should have approximate complexity O(log^2 n).
    ///
    /// Note that no parents have their bounds/height updated directly by this function, instead
    /// [`do_refit`](struct.DynamicBoundingVolumeTree.html#method.do_refit) should be called after
    /// all insert/remove/updates have been performed this frame.
    ///
    /// ### Parameters
    ///
    /// - `value`: The value to insert into the tree.
    ///
    /// ### Returns
    ///
    /// The node index of the inserted value. This value should never change after insertion.
    ///
    pub fn insert(&mut self, value: T) -> usize {
        let fat_bound = value.get_bound_with_margin();
        let value_index = self.values.len();
        self.values.push((0, value));

        // Create a new leaf node for the given value
        let mut new_leaf = Leaf {
            parent: 0,
            value: value_index,
            bound: fat_bound,
        };

        // If the root index is 0, this is the first node inserted, and we can circumvent a lot of
        // checks.
        if self.root_index == 0 {
            self.root_index = self.nodes.len();
            self.nodes.push(Node::Leaf(new_leaf));
            self.values[value_index].0 = self.root_index;
            self.root_index
        } else {
            // Start searching from the root node
            let mut node_index = self.root_index;

            // We will always insert a branch node and the new leaf node, so get 2 new indices
            // into the node list
            let (new_branch_index, new_leaf_index) = self.next_free();
            // We need to tell the value what it's node index is
            self.values[value_index].0 = new_leaf_index;
            // The new leaf will always be a child of the new branch node.
            new_leaf.parent = new_branch_index;

            let mut branch_parent_index = 0;

            loop {
                // If we encounter a leaf node, we've found the place where we want to add the new
                // nodes. The branch node will be inserted into the tree here, and this node will be
                // moved down as the left child of the new branch node, and the new leaf will be the
                // right child.
                let add_branch = match self.nodes[node_index] {
                    Node::Leaf(ref leaf) => {
                        let new_branch = Branch {
                            left: node_index,      // old leaf at the current position is left child
                            right: new_leaf_index, // new leaf node is the right child
                            parent: leaf.parent,   // parent of the branch is the old leaf parent
                            height: 2, // leafs have height 1, so new branch have height 2
                            bound: leaf.bound.union(&new_leaf.bound),
                        };
                        Some((node_index, new_branch))
                    }

                    // If we hit a branch, we compute the surface area of the bounding volumes for
                    // if the new leaf was added to the right or left. Whichever surface area is
                    // lowest will decide which child to go to next.
                    Node::Branch(ref branch) => {
                        let left_bound = get_bound(&self.nodes[branch.left]);
                        let right_bound = get_bound(&self.nodes[branch.right]);
                        let left_area = left_bound.union(&new_leaf.bound).surface_area();
                        let right_area = right_bound.union(&new_leaf.bound).surface_area();
                        if left_area < right_area {
                            node_index = branch.left;
                        } else {
                            node_index = branch.right;
                        }
                        None
                    }

                    Node::Nil => break,
                };

                // time to actually update the tree
                if let Some((leaf_index, branch)) = add_branch {
                    // the old leaf node needs to point to the new branch node as its parent
                    if let Node::Leaf(ref mut n) = self.nodes[leaf_index] {
                        n.parent = new_branch_index;
                    };

                    // if the old leaf node wasn't the root of tree, we update it's parent to point
                    // to the new branch node instead of the old leaf node
                    branch_parent_index = branch.parent;
                    if branch.parent != 0 {
                        if let Node::Branch(ref mut n) = self.nodes[branch.parent] {
                            if n.left == leaf_index {
                                n.left = new_branch_index;
                            } else {
                                n.right = new_branch_index;
                            }
                        }
                    }

                    // insert to new branch and leaf nodes
                    self.nodes[new_branch_index] = Node::Branch(branch);
                    self.nodes[new_leaf_index] = Node::Leaf(new_leaf);

                    // if the leaf node was the root of the tree,
                    // the new root is the new branch node
                    if leaf_index == self.root_index {
                        self.root_index = new_branch_index;
                    }
                    break;
                }
            }

            // mark the new branch nodes parent for bounds/height updating and possible rotation
            if branch_parent_index != 0 {
                self.mark_for_refit(branch_parent_index, 3);
            }

            new_leaf_index
        }
    }

    /// Remove the node with the given node index.
    ///
    /// The reason this function takes the node index and not a reference to the value, is because
    /// the only way to get at the values in the tree is by doing a mutable borrow, making this
    /// function unusable.
    ///
    /// If the given node index points to a non-leaf, this function is effectively a nop.
    /// Else the leaf node and it's parent branch node will be removed, and the leaf nodes sibling
    /// will take the place of the parent branch node in the tree.
    ///
    /// Note that no parents have their bounds/height updated directly by this function, instead
    /// [`do_refit`](struct.DynamicBoundingVolumeTree.html#method.do_refit) should be called after
    /// all insert/remove/updates have been performed this frame.
    ///
    /// This function should have approximate complexity O(log^2 n).
    ///
    /// ### Parameters
    ///
    /// - `node_index`: index of the leaf to remove
    ///
    /// ### Returns
    ///
    /// If a value was removed, the value is returned, otherwise None.
    ///
    pub fn remove(&mut self, node_index: usize) -> Option<T> {
        let (value_index, parent_index) = if let Node::Leaf(ref leaf) = self.nodes[node_index] {
            (leaf.value, leaf.parent)
        } else {
            // If a value points to a non-leaf something has gone wrong,
            // ignore remove and continue with life
            return None;
        };
        // remove from values list and update node list with new value indices
        let (_, value) = self.values.swap_remove(value_index);
        // we only need to update the node for the value that we swapped into the old values place
        if value_index < self.values.len() {
            // should only fail if we just removed the last value
            if let Node::Leaf(ref mut leaf) = self.nodes[self.values[value_index].0] {
                leaf.value = value_index;
            }
        }

        // remove from node list and add index to free list
        self.nodes[node_index] = Node::Nil;
        self.free_list.push(node_index);

        if parent_index != 0 {
            // remove parent branch from node list and add index to free list
            let (parent_parent_index, sibling_index) =
                if let Node::Branch(ref branch) = self.nodes[parent_index] {
                    (
                        branch.parent,
                        if branch.left == node_index {
                            branch.right
                        } else {
                            branch.left
                        },
                    )
                } else {
                    return Some(value);
                };
            self.nodes[parent_index] = Node::Nil;
            self.free_list.push(parent_index);

            // set sibling parent to parent.parent
            match self.nodes[sibling_index] {
                Node::Branch(ref mut branch) => branch.parent = parent_parent_index,
                Node::Leaf(ref mut leaf) => leaf.parent = parent_parent_index,
                Node::Nil => (),
            }

            // if parents parent is 0, the sibling is the last node in the tree and becomes the new
            // root node
            if parent_parent_index == 0 {
                self.root_index = sibling_index;
            } else {
                // else we have a remaining branch, and need to update either left or right to point
                // to the sibling, based on where the old branch node was
                if let Node::Branch(ref mut b) = self.nodes[parent_parent_index] {
                    if b.left == parent_index {
                        b.left = sibling_index;
                    } else {
                        b.right = sibling_index;
                    }
                }

                // mark parents parent for recalculation
                self.mark_for_refit(parent_parent_index, 0);
            }
        } else {
            // if parent was 0, this was the last node in the tree, and the tree is now empty.
            // reset all values.
            self.clear();
        }

        Some(value)
    }

    /// Go through the list of nodes marked for refitting, update their bounds/heights and check if
    /// any of them need to be rotated to new locations.
    ///
    /// This method have worst case complexity O(m * log^2 n), where m is the number of nodes in the
    /// refit list.
    ///
    pub fn do_refit(&mut self) {
        while !self.refit_nodes.is_empty() {
            let (_, node_index) = self.refit_nodes.remove(0);
            self.refit_node(node_index);
        }
    }

    /// Get two new node indices, where nodes can be inserted in the tree.
    ///
    fn next_free(&mut self) -> (usize, usize) {
        (self.take_free(), self.take_free())
    }

    /// Get a new node index, where a node can be inserted in the tree.
    ///
    fn take_free(&mut self) -> usize {
        if self.free_list.is_empty() {
            let index = self.nodes.len();
            self.nodes.push(Node::Nil);
            index
        } else {
            self.free_list.remove(0)
        }
    }

    /// Add the given node to the refitting list.
    ///
    /// The refit list is sorted by the height of the node, and only have the same value
    /// once, any duplicates are rejected. This because we don't want to refit the same node
    /// twice.
    ///
    /// ### Parameters
    ///
    /// - `node_index`: index of the node to do refitting on.
    /// - `min_height`: the minimum height the node has. Used primarily by insertion where we can't
    ///                 be sure that the node has been refitted yet and might have an incorrect
    ///                 height
    ///
    fn mark_for_refit(&mut self, node_index: usize, min_height: u32) {
        let node_height = match self.nodes[node_index] {
            Node::Branch(ref b) => b.height,
            _ => 0,
        };
        let height = max(node_height, min_height);
        let value = (height, node_index);
        match self.refit_nodes.binary_search(&value) {
            Ok(_) => (),
            Err(i) => self.refit_nodes.insert(i, value),
        }
    }

    /// Actually refit a node in the tree. This will check the node for rotation, and if rotated,
    /// will update the bound/height of itself and any other rotated nodes, and also mark its parent
    /// for refitting.
    ///
    fn refit_node(&mut self, node_index: usize) {
        if let Some((parent_index, height)) = self.recalculate_node(node_index) {
            if parent_index != 0 {
                self.mark_for_refit(parent_index, height + 1);
            }
        }

        // Only do rotations occasionally, as they are fairly expensive, and shouldn't be overused.
        // For most scenarios, the majority of shapes will not have moved, so this is fine.
        if rand::thread_rng().gen_range(0..100) < PERFORM_ROTATION_PERCENTAGE {
            self.rotate(node_index);
        }
    }

    /// Recalculate the bound and height of the node.
    ///
    fn recalculate_node(&mut self, node_index: usize) -> Option<(usize, u32)> {
        let (height, bound, parent_index) = {
            let (left_height, left_bound, right_height, right_bound, parent_index) =
                if let Node::Branch(ref branch) = self.nodes[node_index] {
                    (
                        get_height(&self.nodes[branch.left]),
                        get_bound(&self.nodes[branch.left]),
                        get_height(&self.nodes[branch.right]),
                        get_bound(&self.nodes[branch.right]),
                        branch.parent,
                    )
                } else {
                    return None;
                };
            (
                1 + max(left_height, right_height),
                left_bound.union(right_bound),
                parent_index,
            )
        };
        if let Node::Branch(ref mut branch) = self.nodes[node_index] {
            branch.height = height;
            branch.bound = bound;
        }

        Some((parent_index, height))
    }

    /// Check if the node needs to be rotated, and perform the rotation if that is the case.
    ///
    /// ### Parameters:
    ///
    /// - `node_index`: index of the node to check for rotation
    ///
    /// ### Returns
    ///
    /// The parent index of the given node
    ///
    fn rotate(&mut self, node_index: usize) -> Option<usize> {
        let improvement_percentage: <T::Bound as SurfaceArea>::Scalar =
            NumCast::from(SURFACE_AREA_IMPROVEMENT_FOR_ROTATION).unwrap();

        let (left_index, right_index, my_surface_area, parent_index) =
            if let Node::Branch(ref branch) = self.nodes[node_index] {
                (
                    branch.left,
                    branch.right,
                    branch.bound.surface_area(),
                    branch.parent,
                )
            } else {
                return None;
            };

        let left_is_leaf = is_leaf(&self.nodes[left_index]);
        let right_is_leaf = is_leaf(&self.nodes[right_index]);

        // if the node is a grandparent, we can do rotation checks
        if !left_is_leaf || !right_is_leaf {
            let (rot, min_sa) = get_best_rotation(
                &self.nodes,
                left_index,
                right_index,
                my_surface_area,
                left_is_leaf,
                right_is_leaf,
            );

            // we now know which rotation will give us the best surface area
            // only do actual rotation if the surface area is reduced by at least 25%
            if (my_surface_area - min_sa) / my_surface_area > improvement_percentage {
                match rot {
                    // do nothing
                    Rotation::None => (),

                    // swap left child with right left grandchild
                    // right child and node needs to be recalculated
                    Rotation::LeftRightLeft => {
                        let right_left_index = get_left_index(&self.nodes[right_index]);
                        swap(
                            &mut self.nodes,
                            left_index,
                            right_left_index,
                            node_index,
                            right_index,
                        );
                        self.recalculate_node(right_index);
                        self.recalculate_node(node_index);
                    }

                    // swap left child with right right grandchild
                    // right child and node needs to be recalculated
                    Rotation::LeftRightRight => {
                        let right_right_index = get_right_index(&self.nodes[right_index]);
                        swap(
                            &mut self.nodes,
                            left_index,
                            right_right_index,
                            node_index,
                            right_index,
                        );
                        self.recalculate_node(right_index);
                        self.recalculate_node(node_index);
                    }

                    // swap right child with left left grandchild
                    // left child and node needs to be recalculated
                    Rotation::RightLeftLeft => {
                        let left_left_index = get_left_index(&self.nodes[left_index]);
                        swap(
                            &mut self.nodes,
                            left_left_index,
                            right_index,
                            left_index,
                            node_index,
                        );
                        self.recalculate_node(left_index);
                        self.recalculate_node(node_index);
                    }

                    // swap right child with left right grandchild
                    // left child and node needs to be recalculated
                    Rotation::RightLeftRight => {
                        let left_right_index = get_right_index(&self.nodes[left_index]);
                        swap(
                            &mut self.nodes,
                            left_right_index,
                            right_index,
                            left_index,
                            node_index,
                        );
                        self.recalculate_node(left_index);
                        self.recalculate_node(node_index);
                    }

                    // swap left left grandchild with right left grandchild
                    // left child, right child and node needs to be recalculated
                    Rotation::LeftLeftRightLeft => {
                        let left_left_index = get_left_index(&self.nodes[left_index]);
                        let right_left_index = get_left_index(&self.nodes[right_index]);
                        swap(
                            &mut self.nodes,
                            left_left_index,
                            right_left_index,
                            left_index,
                            right_index,
                        );
                        self.recalculate_node(left_index);
                        self.recalculate_node(right_index);
                        self.recalculate_node(node_index);
                    }

                    // swap left left grandchild with right right grandchild
                    // left child, right child and node needs to be recalculated
                    Rotation::LeftLeftRightRight => {
                        let left_left_index = get_left_index(&self.nodes[left_index]);
                        let right_right_index = get_right_index(&self.nodes[right_index]);
                        swap(
                            &mut self.nodes,
                            left_left_index,
                            right_right_index,
                            left_index,
                            right_index,
                        );
                        self.recalculate_node(left_index);
                        self.recalculate_node(right_index);
                        self.recalculate_node(node_index);
                    }
                }
            }
        }

        Some(parent_index)
    }
}

enum Rotation {
    None,
    LeftRightLeft,
    LeftRightRight,
    RightLeftLeft,
    RightLeftRight,
    LeftLeftRightLeft,
    LeftLeftRightRight,
}

/// Swap two nodes in the tree.
///
/// left_parent.`[left,right]` = right_swap
/// right_parent.`[left,right]` = left_swap
/// left_swap.parent = right_parent
/// right_swap.parent = left_parent
///
#[inline]
fn swap<B>(
    nodes: &mut Vec<Node<B>>,
    left_swap_index: usize,
    right_swap_index: usize,
    left_parent_index: usize,
    right_parent_index: usize,
) {
    if let Node::Branch(ref mut left_parent) = nodes[left_parent_index] {
        if left_parent.left == left_swap_index {
            left_parent.left = right_swap_index;
        } else {
            left_parent.right = right_swap_index;
        }
    }

    if let Node::Branch(ref mut right_parent) = nodes[right_parent_index] {
        if right_parent.left == right_swap_index {
            right_parent.left = left_swap_index;
        } else {
            right_parent.right = left_swap_index;
        }
    }

    match nodes[left_swap_index] {
        Node::Branch(ref mut left) => left.parent = right_parent_index,
        Node::Leaf(ref mut left) => left.parent = right_parent_index,
        _ => (),
    }

    match nodes[right_swap_index] {
        Node::Branch(ref mut rl) => rl.parent = left_parent_index,
        Node::Leaf(ref mut rl) => rl.parent = left_parent_index,
        _ => (),
    }
}

/// Calculate the best rotation for a given grandparent node in the tree
#[inline]
fn get_best_rotation<B>(
    nodes: &[Node<B>],
    left_index: usize,
    right_index: usize,
    node_surface_area: <B as SurfaceArea>::Scalar,
    left_is_leaf: bool,
    right_is_leaf: bool,
) -> (Rotation, <B as SurfaceArea>::Scalar)
where
    B: Union<B, Output = B> + SurfaceArea,
{
    let mut rot = Rotation::None;
    let mut min_sa = node_surface_area;

    // we need the left and right child bounds for 4 tests
    let l_bound = get_bound(&nodes[left_index]);
    let r_bound = get_bound(&nodes[right_index]);

    // if the right child is not a leaf, we want to consider swapping its children with
    // either the left child or the left left grandchild
    if !right_is_leaf {
        let (rl_bound, rr_bound) = match nodes[right_index] {
            Node::Branch(ref right) => (
                get_bound(&nodes[right.left]),
                get_bound(&nodes[right.right]),
            ),
            _ => panic!(),
        };

        // check for left child swapped with right left grandchild
        let l_rl_sa = sa(rl_bound, l_bound, rr_bound);
        if l_rl_sa < min_sa {
            rot = Rotation::LeftRightLeft;
            min_sa = l_rl_sa;
        }

        // check for left child swapped with right right grandchild
        let l_rr_sa = sa(rr_bound, l_bound, rl_bound);
        if l_rr_sa < min_sa {
            rot = Rotation::LeftRightRight;
            min_sa = l_rr_sa;
        }

        // check for left left grandchild swapped with either of the right grandchildren
        if !left_is_leaf {
            let (ll_bound, lr_bound) = match nodes[left_index] {
                Node::Branch(ref left) => {
                    (get_bound(&nodes[left.left]), get_bound(&nodes[left.right]))
                }
                _ => panic!(),
            };

            // check for left left grandchild swapped with right left grandchild
            let ll_rl_sa = sa(&rl_bound.union(lr_bound), ll_bound, rr_bound);
            if ll_rl_sa < min_sa {
                rot = Rotation::LeftLeftRightLeft;
                min_sa = ll_rl_sa;
            }

            // check for left left grandchild swapped with right right grandchild
            let ll_rr_sa = sa(&rr_bound.union(lr_bound), rl_bound, ll_bound);
            if ll_rr_sa < min_sa {
                rot = Rotation::LeftLeftRightRight;
                min_sa = ll_rr_sa;
            }

            // we don't need to check for left right grandchild swapped with any of the right
            // grandchildren. this would only result in mirrored trees and have the same cost
            // as other cases, making it redundant work
        }
    }

    // if the left child is not a leaf, we want to consider swapping its children with the
    // right child
    if !left_is_leaf {
        let (ll_bound, lr_bound) = match nodes[left_index] {
            Node::Branch(ref left) => (get_bound(&nodes[left.left]), get_bound(&nodes[left.right])),
            _ => panic!(),
        };

        // check for right child swapped with left left grandchild
        let r_ll_sa = sa(ll_bound, r_bound, lr_bound);
        if r_ll_sa < min_sa {
            rot = Rotation::RightLeftLeft;
            min_sa = r_ll_sa;
        }

        // check for right child swapped with left right grandchild
        let r_lr_sa = sa(lr_bound, r_bound, ll_bound);
        if r_lr_sa < min_sa {
            rot = Rotation::RightLeftRight;
            min_sa = r_lr_sa;
        }
    }

    (rot, min_sa)
}

/// Calculate the surface area of 3 combined bounding volumes, a.union(b.union(c)).
#[inline]
fn sa<B>(a: &B, b: &B, c: &B) -> <B as SurfaceArea>::Scalar
where
    B: Union<B, Output = B> + SurfaceArea,
{
    a.union(&b.union(c)).surface_area()
}

#[inline]
fn get_left_index<B>(node: &Node<B>) -> usize {
    match *node {
        Node::Branch(ref b) => b.left,
        _ => panic!(),
    }
}

#[inline]
fn get_right_index<B>(node: &Node<B>) -> usize {
    match *node {
        Node::Branch(ref b) => b.right,
        _ => panic!(),
    }
}

#[inline]
fn is_leaf<B>(node: &Node<B>) -> bool {
    matches!(*node, Node::Leaf(_))
}

/// Get the height of the node, regardless of node type. Leafs have height 1, nil height 0.
///
#[inline]
fn get_height<B>(node: &Node<B>) -> u32 {
    match *node {
        Node::Branch(ref branch) => branch.height,
        Node::Leaf(_) => 1,
        Node::Nil => 0,
    }
}

/// Get the bound of the node. Will panic if node is nil.
///
#[inline]
fn get_bound<B>(node: &Node<B>) -> &B {
    match *node {
        Node::Branch(ref branch) => &branch.bound,
        Node::Leaf(ref leaf) => &leaf.bound,
        Node::Nil => panic!(),
    }
}
