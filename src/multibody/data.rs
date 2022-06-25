use cxx::UniquePtr;
use nalgebra::geometry::{Rotation3, UnitQuaternion, Translation3, Isometry3};
use nalgebra::base::{Vector3, Matrix3};
use crate::multibody::Model;

#[cxx::bridge(namespace = "pinocchio")]
pub mod ffi_data {
    unsafe extern "C++" {
        include!("pinocchio-rust/multibody/data.hpp");

        type Model = crate::multibody::ffi_model::Model;
        type Data;
        fn createData(model: &UniquePtr<Model>) -> UniquePtr<Data>;
        fn cloneData(data: &UniquePtr<Data>) -> UniquePtr<Data>;
        fn nframes(data: &UniquePtr<Data>) -> u32;
        fn njoints(data: &UniquePtr<Data>) -> u32;
        fn frameTranslation(data : &UniquePtr<Data>, frame_id: &u32) -> UniquePtr<CxxVector<f64>>;
        fn frameRotation(data : &UniquePtr<Data>, frame_id: &u32) -> UniquePtr<CxxVector<f64>>;
        fn jointTranslation(data : &UniquePtr<Data>, joint_id: &u32) -> UniquePtr<CxxVector<f64>>;
        fn jointRotation(data : &UniquePtr<Data>, joint_id: &u32) -> UniquePtr<CxxVector<f64>>;
    }
}


pub struct Data {
    pub ptr: UniquePtr<ffi_data::Data>,
}


impl Data {
    pub fn new(model: &Model) -> Data {
        Data { ptr: ffi_data::createData(&model.ptr) }
    }

    pub fn clone(&self) -> Data {
        Data { ptr: ffi_data::cloneData(&self.ptr) }
    }

    pub fn nframes(&self) -> u32 {
        ffi_data::nframes(&self.ptr)
    }

    pub fn njoints(&self) -> u32 {
        ffi_data::njoints(&self.ptr)
    }

    // pub fn frame_placement(&self, frame_id: u32) -> Option<Isometry3<f64>> {
    //     if frame_id < self.nframes() {
    //         let trans = ffi_data::frameTranslation(&self.ptr, &frame_id);
    //         let rot= ffi_data::frameRotation(&self.ptr, &frame_id);
    //         let trans = crate::math::stdvec_to_vector3(&trans);
    //         let rot = crate::math::stdvec_to_matrix3(&rot);
    //         let trans = Translation3::from(trans);
    //         let rot = Rotation3::from(rot);
    //         let rot: quat = UnitQuaternion::from_matrix()
    //         Some(Isometry3::from_parts(trans, rot));
    //     }
    //     else {
    //         None
    //     }
    // }
}