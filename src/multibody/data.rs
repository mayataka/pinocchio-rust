use cxx::UniquePtr;
use nalgebra::geometry::{Rotation3, UnitQuaternion, Translation3, Isometry3};
use nalgebra::base::{Vector3, Matrix3};
use crate::multibody::Model;
use crate::math::cxxvec;

#[cxx::bridge(namespace = "pinocchio")]
pub mod ffi_data {
    unsafe extern "C++" {
        include!("pinocchio-rust/multibody/data.hpp");

        type Model = crate::multibody::ffi_model::Model;
        type Data;
        fn createData(model: &UniquePtr<Model>) -> UniquePtr<Data>;
        fn cloneData(data: &UniquePtr<Data>) -> UniquePtr<Data>;
        fn nframesInData(data: &UniquePtr<Data>) -> u32;
        fn njointsInData(data: &UniquePtr<Data>) -> u32;
        // fn frameTranslation(data : &UniquePtr<Data>, frame_id: &u32) -> UniquePtr<CxxVector<f64>>;
        // fn frameRotation(data : &UniquePtr<Data>, frame_id: &u32) -> UniquePtr<CxxVector<f64>>;
        // fn jointTranslation(data : &UniquePtr<Data>, joint_id: &u32) -> UniquePtr<CxxVector<f64>>;
        // fn jointRotation(data : &UniquePtr<Data>, joint_id: &u32) -> UniquePtr<CxxVector<f64>>;
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
        ffi_data::nframesInData(&self.ptr)
    }

    pub fn njoints(&self) -> u32 {
        ffi_data::njointsInData(&self.ptr)
    }

    // pub unsafe fn frame_translation(&self, frame_id: u32) -> Option<Vector3<f64>> {
    //     if frame_id < self.nframes() {
    //         let trans= ffi_data::frameTranslation(&self.ptr, &frame_id);
    //         Some(cxxvec::cxxvec_to_vector3(&trans).unwrap())
    //     }
    //     else {
    //         None
    //     }
    // }

    // pub unsafe fn frame_rotation(&self, frame_id: u32) -> Option<Matrix3<f64>> {
    //     if frame_id < self.nframes() {
    //         let rot= ffi_data::frameRotation(&self.ptr, &frame_id);
    //         Some(cxxvec::cxxvec_to_matrix3(&rot).unwrap())
    //     }
    //     else {
    //         None
    //     }
    // }

    // pub unsafe fn joint_translation(&self, joint_id: u32) -> Option<Vector3<f64>> {
    //     if joint_id < self.njoints() {
    //         let trans= ffi_data::jointTranslation(&self.ptr, &joint_id);
    //         Some(cxxvec::cxxvec_to_vector3(&trans).unwrap())
    //     }
    //     else {
    //         None
    //     }
    // }

    // pub unsafe fn joint_rotation(&self, joint_id: u32) -> Option<Matrix3<f64>> {
    //     if joint_id < self.njoints() {
    //         let rot= ffi_data::jointRotation(&self.ptr, &joint_id);
    //         Some(cxxvec::cxxvec_to_matrix3(&rot).unwrap())
    //     }
    //     else {
    //         None
    //     }
    // }

}