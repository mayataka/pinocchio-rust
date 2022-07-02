use cxx;
extern crate nalgebra as na;
use crate::multibody::Model;
use crate::multibody::Data;
use crate::algorithm::ReferenceFrame;

#[cxx::bridge(namespace = "pinocchio")]
pub mod ffi_jacobian {
    unsafe extern "C++" {
        include!("pinocchio-rust/algorithm/jacobian.hpp");

        type Model = crate::multibody::ffi_model::Model;
        type Data = crate::multibody::ffi_data::Data;
        fn computeJointJacobians(model: &UniquePtr<Model>, data: &mut UniquePtr<Data>, q: &[f64]);
        fn getJointJacobian(model: &UniquePtr<Model>, data : &mut UniquePtr<Data>, joint_id: u32, rf: u32, J: &mut [f64]);
        fn computeJointJacobian(model: &UniquePtr<Model>, data : &mut UniquePtr<Data>, q: &[f64], joint_id: u32, J: &mut [f64]);
        fn computeJointJacobiansTimeVariation(model: &UniquePtr<Model>, data: &mut UniquePtr<Data>, q: &[f64], v: &[f64]);
        fn getJointJacobianTimeVariation(model: &UniquePtr<Model>, data : &mut UniquePtr<Data>, joint_id: u32, rf: u32, dJ: &mut [f64]);
        fn jointJacobianSize(model: &UniquePtr<Model>) -> Vec<u32>;
        fn jointJacobianTimeVaryationSize(model: &UniquePtr<Model>) -> Vec<u32>;
    }
}


pub fn compute_joint_jacobians(model: &Model, data: &mut Data, q: &na::DVector<f64>) -> Result<(), String> {
    if q.len() == model.nq() {
        let q = q.as_slice();
        ffi_jacobian::computeJointJacobians(&model.ptr, &mut data.ptr, q);
        Ok(())
    }
    else {
        Err("Invalid size of q".to_string())
    }
}

pub fn get_joint_jacobian(model: &Model, data: &mut Data, joint_id: usize, rf: ReferenceFrame, jac: &mut na::DMatrix<f64>) -> Result<(), String> {
    let size = ffi_jacobian::jointJacobianSize(&model.ptr);
    let nrows = size[0] as usize;
    let ncols = size[1] as usize;
    if joint_id < model.njoints() && jac.nrows() == nrows && jac.ncols() == ncols {
        let joint_id = joint_id as u32;
        let rf = rf.to_u32();
        let jac = jac.as_mut_slice();
        ffi_jacobian::getJointJacobian(&model.ptr, &mut data.ptr, joint_id, rf, jac);
        Ok(())
    }
    else {
        Err("Invalid joint_id or size of jac".to_string())
    }
}

pub fn compute_joint_jacobian(model: &Model, data: &mut Data, q: &na::DVector<f64>, joint_id: usize, jac: &mut na::DMatrix<f64>) -> Result<(), String> {
    let size = ffi_jacobian::jointJacobianSize(&model.ptr);
    let nrows = size[0] as usize;
    let ncols = size[1] as usize;
    if joint_id < model.njoints() && jac.nrows() == nrows && jac.ncols() == ncols {
        let q = q.as_slice();
        let joint_id = joint_id as u32;
        let jac = jac.as_mut_slice();
        ffi_jacobian::computeJointJacobian(&model.ptr, &mut data.ptr, q, joint_id, jac);
        Ok(())
    }
    else {
        Err("Invalid joint_id or size of jac".to_string())
    }
}

pub fn compute_joint_jacobians_time_variation(model: &Model, data: &mut Data, q: &na::DVector<f64>, v: &na::DVector<f64>) -> Result<(), String> {
    if q.len() == model.nq() && v.len() == model.nv() {
        let q = q.as_slice();
        let v = v.as_slice();
        ffi_jacobian::computeJointJacobiansTimeVariation(&model.ptr, &mut data.ptr, q, v);
        Ok(())
    }
    else {
        Err("Invalid sizes in q or v".to_string())
    }
}

pub fn get_joint_jacobian_time_variation(model: &Model, data: &mut Data, joint_id: usize, rf: ReferenceFrame, djac: &mut na::DMatrix<f64>) -> Result<(), String> {
    let size = ffi_jacobian::jointJacobianTimeVaryationSize(&model.ptr);
    let nrows = size[0] as usize;
    let ncols = size[1] as usize;
    if joint_id < model.njoints() && djac.nrows() == nrows && djac.ncols() == ncols {
        let joint_id = joint_id as u32;
        let rf = rf.to_u32();
        let djac = djac.as_mut_slice();
        ffi_jacobian::getJointJacobianTimeVariation(&model.ptr, &mut data.ptr, joint_id, rf, djac);
        Ok(())
    }
    else {
        Err("Invalid joint_id or size of djac".to_string())
    }
}