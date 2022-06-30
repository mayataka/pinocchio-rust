use std::vec::Vec;
use cxx;
use nalgebra as na;
use crate::multibody::Model;

#[cxx::bridge(namespace = "pinocchio")]
pub mod ffi_joint_configuraiton {
    unsafe extern "C++" {
        include!("pinocchio-rust/algorithm/joint_configuration.hpp");

        type Model = crate::multibody::ffi_model::Model;
        fn integrate(model: &UniquePtr<Model>, q: &[f64], v: &[f64]) -> Vec<f64>;
        fn interpolate(model: &UniquePtr<Model>, q0: &[f64], q1: &[f64], u: f64) -> Vec<f64>;
        fn difference(model: &UniquePtr<Model>, q0: &[f64], q1: &[f64]) -> Vec<f64>;
        fn randomConfiguration(model: &UniquePtr<Model>, lower_limits: &[f64], upper_limit: &[f64]) -> Vec<f64>;
        fn neutral(model: &UniquePtr<Model>) -> Vec<f64>;
    }
}


pub fn integrate(model: &Model, q: &na::DVector<f64>, v: &na::DVector<f64>) -> Option<na::DVector<f64>> {
    if q.len() == model.nq() && v.len() == model.nv() {
        let q = q.as_slice();
        let v = v.as_slice();
        let qout = ffi_joint_configuraiton::integrate(&model.ptr, q, v);
        Some(na::DVector::from_vec(qout))
    }
    else {
        None
    }
}

pub fn interpolate(model: &Model, q0: &na::DVector<f64>, q1: &na::DVector<f64>, u: f64) -> Option<na::DVector<f64>> {
    if q0.len() == model.nq() && q1.len() == model.nq() && 0.0 <= u && u <= 1.0 {
        let q0 = q0.as_slice();
        let q1 = q1.as_slice();
        let qout = ffi_joint_configuraiton::interpolate(&model.ptr, q0, q1, u);
        Some(na::DVector::from_vec(qout))
    }
    else {
        None
    }
}

pub fn difference(model: &Model, q0: &na::DVector<f64>, q1: &na::DVector<f64>) -> Option<na::DVector<f64>> {
    if q1.len() == model.nq() && q1.len() == model.nq() {
        let q0 = q0.as_slice();
        let q1 = q1.as_slice();
        let qout = ffi_joint_configuraiton::difference(&model.ptr, q0, q1);
        Some(na::DVector::from_vec(qout))
    }
    else {
        None
    }
}

pub fn random_configuration(model: &Model, lower_limit: &na::DVector<f64>, upper_limit: &na::DVector<f64>) -> Option<na::DVector<f64>> {
    if lower_limit.len() == model.nq() && upper_limit.len() == model.nq() {
        let lower_limit = lower_limit.as_slice();
        let upper_limit = upper_limit.as_slice();
        let qout = ffi_joint_configuraiton::randomConfiguration(&model.ptr, lower_limit, upper_limit);
        Some(na::DVector::from_vec(qout))
    }
    else {
        None
    }
}

pub fn neutral(model: &Model) -> na::DVector<f64> {
    let qout = ffi_joint_configuraiton::neutral(&model.ptr);
    na::DVector::from_vec(qout)
}
