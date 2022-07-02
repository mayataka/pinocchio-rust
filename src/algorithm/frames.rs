use cxx;
extern crate nalgebra as na;
use crate::multibody::Model;
use crate::multibody::Data;
use crate::algorithm::ReferenceFrame;

#[cxx::bridge(namespace = "pinocchio")]
pub mod ffi_frames {
    unsafe extern "C++" {
        include!("pinocchio-rust/algorithm/frames.hpp");

        type Model = crate::multibody::ffi_model::Model;
        type Data = crate::multibody::ffi_data::Data;
        fn updateFramePlacements(model: &UniquePtr<Model>, data: &mut UniquePtr<Data>);
        fn framesForwardKinematics(model: &UniquePtr<Model>, data: &mut UniquePtr<Data>, q: &[f64]);
        fn getFrameVelocity(model: &UniquePtr<Model>, data : &UniquePtr<Data>, frame_id: u32, rf: u32, vel: &mut [f64]);
        fn getFrameAcceleration(model: &UniquePtr<Model>, data : &UniquePtr<Data>, frame_id: u32, rf: u32, acc: &mut [f64]);
        fn getFrameClassicalAcceleration(model: &UniquePtr<Model>, data : &UniquePtr<Data>, frame_id: u32, rf: u32, acc: &mut [f64]);
        fn getFrameJacobian(model: &UniquePtr<Model>, data : &mut UniquePtr<Data>, frame_id: u32, rf: u32, J: &mut [f64]);
        fn computeFrameJacobian(model: &UniquePtr<Model>, data : &mut UniquePtr<Data>, q: &[f64], frame_id: u32, rf: u32, J: &mut [f64]);
        fn getFrameJacobianTimeVariation(model: &UniquePtr<Model>, data : &mut UniquePtr<Data>, frame_id: u32, rf: u32, dJ: &mut [f64]);
        fn frameJacobianSize(model: &UniquePtr<Model>) -> Vec<u32>;
        fn frameJacobianTimeVaryationSize(model: &UniquePtr<Model>) -> Vec<u32>;
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

pub fn get_frame_velocity(model: &Model, data: &Data, frame_id: usize, rf: ReferenceFrame) -> Option<na::Vector6<f64>> {
    if frame_id < model.nframes() {
        let frame_id = frame_id as u32;
        let rf = rf.to_u32();
        let mut vel = na::Vector6::<f64>::zeros();
        let vel_mut_slice = vel.as_mut_slice();
        ffi_frames::getFrameVelocity(&model.ptr, &data.ptr, frame_id, rf, vel_mut_slice);
        Some(vel)
    }
    else {
        None
    }
}

pub fn get_frame_acceleration(model: &Model, data: &Data, frame_id: usize, rf: ReferenceFrame) -> Option<na::Vector6<f64>> {
    if frame_id < model.nframes() {
        let frame_id = frame_id as u32;
        let rf = rf.to_u32();
        let mut acc = na::Vector6::<f64>::zeros();
        let acc_mut_slice = acc.as_mut_slice();
        ffi_frames::getFrameAcceleration(&model.ptr, &data.ptr, frame_id, rf, acc_mut_slice);
        Some(acc)
    }
    else {
        None
    }
}

pub fn get_frame_classical_acceleration(model: &Model, data: &Data, frame_id: usize, rf: ReferenceFrame) -> Option<na::Vector6<f64>> {
    if frame_id < model.nframes() {
        let frame_id = frame_id as u32;
        let rf = rf.to_u32();
        let mut acc = na::Vector6::<f64>::zeros();
        let acc_mut_slice = acc.as_mut_slice();
        ffi_frames::getFrameClassicalAcceleration(&model.ptr, &data.ptr, frame_id, rf, acc_mut_slice);
        Some(acc)
    }
    else {
        None
    }
}

pub fn get_frame_jacobian(model: &Model, data: &mut Data, frame_id: usize, rf: ReferenceFrame, jac: &mut na::DMatrix<f64>) -> Result<(), String> {
    let size = ffi_frames::frameJacobianSize(&model.ptr);
    let nrows = size[0] as usize;
    let ncols = size[1] as usize;
    if frame_id < model.nframes() && jac.nrows() == nrows && jac.ncols() == ncols {
        let frame_id = frame_id as u32;
        let rf = rf.to_u32();
        let jac = jac.as_mut_slice();
        ffi_frames::getFrameJacobian(&model.ptr, &mut data.ptr, frame_id, rf, jac);
        Ok(())
    }
    else {
        Err("Invalid frame_id or size of jac".to_string())
    }
}

pub fn compute_frame_jacobian(model: &Model, data: &mut Data, q: &na::DVector<f64>, frame_id: usize, rf: ReferenceFrame, jac: &mut na::DMatrix<f64>) -> Result<(), String> {
    let size = ffi_frames::frameJacobianSize(&model.ptr);
    let nrows = size[0] as usize;
    let ncols = size[1] as usize;
    if frame_id < model.nframes() && q.len() == model.nq() && jac.nrows() == nrows && jac.ncols() == ncols {
        let q = q.as_slice();
        let frame_id = frame_id as u32;
        let rf = rf.to_u32();
        let jac = jac.as_mut_slice();
        ffi_frames::computeFrameJacobian(&model.ptr, &mut data.ptr, q, frame_id, rf, jac);
        Ok(())
    }
    else {
        Err("Invalid frame_id or size of jac".to_string())
    }
}

pub fn get_frame_jacobian_time_variation(model: &Model, data: &mut Data, frame_id: usize, rf: ReferenceFrame, djac: &mut na::DMatrix<f64>) -> Result<(), String> {
    let size = ffi_frames::frameJacobianTimeVaryationSize(&model.ptr);
    let nrows = size[0] as usize;
    let ncols = size[1] as usize;
    if frame_id < model.nframes() && djac.nrows() == nrows && djac.ncols() == ncols {
        let frame_id = frame_id as u32;
        let rf = rf.to_u32();
        let djac = djac.as_mut_slice();
        ffi_frames::getFrameJacobianTimeVariation(&model.ptr, &mut data.ptr, frame_id, rf, djac);
        Ok(())
    }
    else {
        Err("Invalid frame_id or size of djac".to_string())
    }
}