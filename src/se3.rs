use nalgebra::{Vector3, Matrix3};

struct SE3 {
  pub translation: Vector3,
  pub rotation: Matrix3, 
}

impl SE3 {
    pub fn new() -> SE3 {
        SE3 { translation: Vector3::zero(), rotation: Matrix3::identity() }
    }

    pub fn clone(&self) -> Data {
        Data { ptr: ffi_data::cloneData(&self.ptr) }
    }
}