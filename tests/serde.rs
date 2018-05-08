#![cfg(feature = "serde")]

extern crate cgmath;
extern crate serde;

use cgmath::Point1;
use serde::Serialize;

fn has_serialize<S: Serialize>() -> bool {
    true
}

#[test]
fn test_feature_enabled() {
    assert!(has_serialize::<Point1<f32>>());
}
