extern crate collision;

use collision::PlaneBound;

fn _box(_: Box<dyn PlaneBound<f32>>) {}

#[test]
fn bound_box() {}
