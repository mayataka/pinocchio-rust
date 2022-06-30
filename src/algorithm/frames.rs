use cxx;
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
        fn getFrameVelocity(model: &UniquePtr<Model>, data : &UniquePtr<Data>, frame_id: u32) -> Vec<f64>;
        fn getFrameAcceleration(model: &UniquePtr<Model>, data : &UniquePtr<Data>, frame_id: u32) -> Vec<f64>;
        fn getFrameClassicalAcceleration(model: &UniquePtr<Model>, data : &UniquePtr<Data>, frame_id: u32) -> Vec<f64>;
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

pub fn get_frame_velocity(model: &Model, data: &Data, frame_id: usize) -> Option<na::Vector6<f64>> {
    if frame_id < model.nframes() {
        let frame_id = frame_id as u32;
        let vel = ffi_frames::getFrameVelocity(&model.ptr, &data.ptr, frame_id);
        Some(na::Vector6::from_vec(vel))
    }
    else {
        None
    }
}

pub fn get_frame_acceleration(model: &Model, data: &Data, frame_id: usize) -> Option<na::Vector6<f64>> {
    if frame_id < model.nframes() {
        let frame_id = frame_id as u32;
        let acc = ffi_frames::getFrameAcceleration(&model.ptr, &data.ptr, frame_id);
        Some(na::Vector6::from_vec(acc))
    }
    else {
        None
    }
}

pub fn get_frame_classical_acceleration(model: &Model, data: &Data, frame_id: usize) -> Option<na::Vector6<f64>> {
    if frame_id < model.nframes() {
        let frame_id = frame_id as u32;
        let acc = ffi_frames::getFrameClassicalAcceleration(&model.ptr, &data.ptr, frame_id);
        Some(na::Vector6::from_vec(acc))
    }
    else {
        None
    }
}