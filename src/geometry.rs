use cgmath::BaseNum;

pub trait SurfaceArea<S: BaseNum> {
    fn surface_area(&self) -> S;
}
