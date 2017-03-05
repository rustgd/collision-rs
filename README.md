# collision-rs

[![Build Status](https://travis-ci.org/csherratt/collision.svg?branch=master)](https://travis-ci.org/csherratt/collision-rs)
[![Documentation](https://docs.rs/collision/badge.svg)](https://docs.rs/collision)
[![Version](https://img.shields.io/crates/v/collision.svg)](https://crates.io/crates/collision)
[![License](https://img.shields.io/crates/l/collision.svg)](https://github.com/csherratt/collision-rs/blob/master/LICENSE)
[![Downloads](https://img.shields.io/crates/d/collision.svg)](https://crates.io/crates/collision)

This library is an extension of [cgmath](https://crates.io/crates/cgmath) that provides collision detection primitives.

The library provides:

- a generic ray: `Ray`
- a plane type: `Plane`
- a view frustum: `Frustum`
- axis-aligned bounding boxes: `Aabb2`, `Aabb3`
- oriented bounding boxes: `Obb2`, `Obb3`
- collision primitives: `Sphere`, `Cylinder`

Not all of the functionality has been implemented yet, and the existing code
is not fully covered by the testsuite. If you encounter any mistakes or
omissions please let me know by posting an issue, or even better: send me a
pull request with a fix.

## Contributing

Pull requests are most welcome, especially in the realm of performance
enhancements and fixing any mistakes I may have made along the way. Unit tests
and benchmarks are also required, so help on that front would be most
appreciated.
