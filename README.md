# collision-rs

[![Build Status](https://travis-ci.org/kvark/collision.svg?branch=master)](https://travis-ci.org/kvark/collision-rs)
[![Documentation](https://docs.rs/collision/badge.svg)](https://docs.rs/collision)
[![Version](https://img.shields.io/crates/v/collision.svg)](https://crates.io/crates/collision)
[![License](https://img.shields.io/crates/l/collision.svg)](https://github.com/kvark/collision-rs/blob/master/LICENSE)
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

## History

This library was originally developed by authors listed below. However, the 
github account with this project was deleted. 
Now it has been reestablished by developers of [Amethyst engine](https://github.com/amethyst/)
### Authors
- Brendan Zabarauskas
- Brian Heylin
- Colin Sherratt
- Dzmitry Malyshau
- Erick Tryzelaar
- Luqman Aden
- Maik Klein
- Mikko Perttunen
- Pierre Krieger
- Tomasz Stachowiak

## Contributing

Use [CONTRIBUTING.md](https://github.com/kvark/collision-rs/blob/master/CONTRIBUTING.md) for futher information.

Pull requests are most welcome, especially in the realm of performance
enhancements and fixing any mistakes. Unit tests and benchmarks are also 
required, so help on that front would be most appreciated.
