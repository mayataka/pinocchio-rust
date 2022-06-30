extern crate nalgebra as na;
use crate::multibody::Model;
use crate::multibody::Data;

#[cxx::bridge(namespace = "pinocchio")]
pub mod ffi_frames {
    unsafe extern "C++" {
        include!("pinocchio-rust/algorithm/frames.hpp");

        type Model = crate::multibody::ffi_model::Model;
        type Data = crate::multibody::ffi_data::Data;
        fn updateFramePlacements(model: &UniquePtr<Model>, data: &mut UniquePtr<Data>);
        fn framesForwardKinematics(model: &UniquePtr<Model>, data: &mut UniquePtr<Data>, q: &[f64]);
    }
}


pub fn update_frame_placements(model: &Model, data: &mut Data) {
    ffi_frames::updateFramePlacements(&model.ptr, &mut data.ptr);
}

pub fn frames_forward_kinematics(model: &Model, data: &mut Data, q: &na::DVector<f64>) -> Result<(), String> {
    if q.len() == model.nq() {
        let q = q.as_slice();
        ffi_frames::framesForwardKinematics(&model.ptr, &mut data.ptr, q);
        Ok(())
    }
    else {
        Err("Invalid size of q".to_string())
    }
}