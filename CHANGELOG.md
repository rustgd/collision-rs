## Change Log

### v0.15.1
  - Add Default implementation for DynamicBoundingVolumeTree

### v0.15
  - Add Quad primitive for 3D
  - Add Line primitive for 2D
  - Fix: Sphere discrete intersection test compared squared radii with non-squared combined radii
  - Expose tolerance settings on GJK/EPA (BREAKING CHANGE: Changed generic type parameters on GJK)
  - Fix: Expose dimensions of Rectangle/Cuboid
  - Fix: Complex time of impact would ignore later primitive pairs if one pair did not collide

### v0.14
  - Upgrade to cgmath 0.16

### v0.13
  - Fix: Exactly overlapping identical shapes caused degeneracy in GJK.
  - Fix: Aabb Point containment overflowed for unsigned BaseNum.
  - Added external access to faces in ConvexPolyhedron.
  - Added new collision primitives:
    * Cylinder (3D)
    * Capsule (3D)
  - Make it possible to be abstract over bounding volumes
  - Removed the SupportFunction trait, incorporating into the Primitive trait
  - Renamed the Bound trait to PlaneBound
  - Added TreeValueWrapped, a utility type for use with the DBVT   

### v0.12 
  - New dynamic bounding volume tree for use with spatial querying etc
  - Added collision primitives:
    * Particle (2D/3D)
    * Rectangle (2D)
    * Circle (2D)
    * ConvexPolygon (2D)
    * Cuboid (3D)
    * Sphere (3D)
    * ConvexPolyhedron (3D)
  - Added wrappers for collision primitives
    * Primitive2 (2D)
    * Primitive3 (3D)
  - Added support functions and bounding volume handling for all primitives
  - Added ray intersection testing for all primitives, both without and with object-to-world transformations
  - Added collision detection algorithms for broad phase collision detection:
    * Brute force (compare all bounding volume pairs)
    * Sweep and Prune (sorts volumes along a sweep axis, and compares bounding volumes where there is an overlap in that 
      axis)
  - Added algorithms that use GJK (Gilbert-Johnson-Keerthi) algorithm, and also EPA (Expanding Polytope Algorithm) for 
    additional contact data:
    * Convex primitive intersection testing
    * Convex primitive distance computation
    * Convex primitive time of impact intersection testing
    * Basic support for composite primitives
  - Reorganised code tree
  - Added unit tests and benchmarks 
