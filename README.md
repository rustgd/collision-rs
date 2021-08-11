# collision-rs

[![CI](https://github.com/rustgd/collision-rs/actions/workflows/ci.yml/badge.svg)](https://github.com/rustgd/collision-rs/actions/workflows/ci.yml)
[![Documentation](https://docs.rs/collision/badge.svg)](https://docs.rs/collision)
[![Version](https://img.shields.io/crates/v/collision.svg)](https://crates.io/crates/collision)
[![License](https://img.shields.io/crates/l/collision.svg)](https://github.com/rustgd/collision-rs/blob/master/LICENSE)
[![Downloads](https://img.shields.io/crates/d/collision.svg)](https://crates.io/crates/collision)
[![Join the chat](https://badges.gitter.im/collision-rs/Lobby.svg)](https://gitter.im/collision-rs/Lobby)

This library is an extension of [cgmath](https://crates.io/crates/cgmath) that provides collision detection primitives,
bounding volumes and collision detection algorithms.

The library provides:

- a generic ray: `Ray`
- a plane type: `Plane`
- a view frustum: `Frustum`
- axis-aligned bounding boxes: `Aabb2`, `Aabb3`
- oriented bounding boxes: `Obb2`, `Obb3`
- additional bounding volumes: `Sphere`, `Cylinder`
- collision primitives: `Sphere`, `Circle`, `Rectangle`, `Cuboid`, `Particle`, `Convex Polygon`, `Convex Polyhedra`
- a dynamic bounding volume tree (`DBVT`)
- broad phase collision detection: `Brute Force`, `Sweep and Prune`
- discrete narrow phase collision detection: `GJK` (including `EPA` for manifold computation)
- continuous narrow phase collision detection: `GJK`
- convex shape distance computation: `GJK`

Not all of the functionality has been implemented yet, and the existing code
is not fully covered by the testsuite. If you encounter any mistakes or
omissions please let us know by posting an issue, or even better: send us a
pull request with a fix.

## License

Licensed under either of

 * Apache License, Version 2.0, ([LICENSE-APACHE](LICENSE-APACHE) or http://www.apache.org/licenses/LICENSE-2.0)
 * MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.

## Contributing

Use [CONTRIBUTING.md](https://github.com/rustgd/collision-rs/blob/master/CONTRIBUTING.md) for further information.

Pull requests are most welcome, especially in the realm of performance
enhancements and fixing any mistakes. Unit tests and benchmarks are also 
required, so help on that front would be most appreciated.

Unless you explicitly state otherwise, any contribution intentionally submitted
for inclusion in the work by you, as defined in the Apache-2.0 license, shall be dual licensed as above, without any
additional terms or conditions.
