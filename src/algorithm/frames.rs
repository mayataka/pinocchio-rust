use std::vec::Vec;
use cxx::{UniquePtr, CxxVector};
use nalgebra::{Vector3, Matrix3, DVector, DMatrix};
use crate::multibody::Model;
use crate::multibody::Data;
use crate::math::cxxvec;

#[cxx::bridge(namespace = "pinocchio")]
pub mod ffi_frames {
    unsafe extern "C++" {
        include!("pinocchio-rust/algorithm/frames.hpp");

        type Model = crate::multibody::ffi_model::Model;
        type Data = crate::multibody::ffi_data::Data;
        fn updateFramePlacements(model: &UniquePtr<Model>, data: &mut UniquePtr<Data>);
        fn framesForwardKinematics(model: &UniquePtr<Model>, data: &mut UniquePtr<Data>, q: &Vec<f64>);
    }
}


pub fn update_frame_placements(model: &Model, data: &mut Data) {
    ffi_frames::updateFramePlacements(&model.ptr, &mut data.ptr);
}

pub fn frames_forward_kinematics(model: &Model, data: &mut Data, q: &Vec<f64>) {
    ffi_frames::framesForwardKinematics(&model.ptr, &mut data.ptr, &q);
}