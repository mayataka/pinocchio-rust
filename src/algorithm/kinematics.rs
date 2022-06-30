use cxx;
use nalgebra as na;
use crate::multibody::Model;
use crate::multibody::Data;

#[cxx::bridge(namespace = "pinocchio")]
pub mod ffi_kinematics {
    unsafe extern "C++" {
        include!("pinocchio-rust/algorithm/kinematics.hpp");

        type Model = crate::multibody::ffi_model::Model;
        type Data = crate::multibody::ffi_data::Data;
        fn forwardKinematicsPositionLevel(model: &UniquePtr<Model>, data: &mut UniquePtr<Data>, 
                                          q: &[f64]);
        fn forwardKinematicsVelocityLevel(model: &UniquePtr<Model>, data: &mut UniquePtr<Data>, 
                                          q: &[f64], v: &[f64]);
        fn forwardKinematicsAccelerationLevel(model: &UniquePtr<Model>, data: &mut UniquePtr<Data>, 
                                              q: &[f64], v: &[f64], a: &[f64]);
    }
}

pub fn forward_kinematics_position_level(model: &Model, data: &mut Data, 
                                         q: &na::DVector<f64>) -> Result<(), String> {
    if q.len() == model.nq() {
        let q = q.as_slice();
        ffi_kinematics::forwardKinematicsPositionLevel(&model.ptr, &mut data.ptr, q);
        Ok(())
    }
    else {
        Err("Invalid size of q".to_string())
    }
}

pub fn forward_kinematics_velocity_level(model: &Model, data: &mut Data, 
                                         q: &na::DVector<f64>, v: &na::DVector<f64>) -> Result<(), String> {
    if q.len() == model.nq() && v.len() == model.nv() {
        let q = q.as_slice();
        let v = v.as_slice();
        ffi_kinematics::forwardKinematicsVelocityLevel(&model.ptr, &mut data.ptr, q, v);
        Ok(())
    }
    else {
        Err("Invalid sizes in q or v".to_string())
    }
}

pub fn forward_kinematics_acceleration_level(model: &Model, data: &mut Data, 
                                             q: &na::DVector<f64>, v: &na::DVector<f64>, a: &na::DVector<f64>) -> Result<(), String> {
    if q.len() == model.nq() && v.len() == model.nv() && a.len() == model.nv() {
        let q = q.as_slice();
        let v = v.as_slice();
        let a = a.as_slice();
        ffi_kinematics::forwardKinematicsAccelerationLevel(&model.ptr, &mut data.ptr, q, v, a);
        Ok(())
    }
    else {
        Err("Invalid sizes in q, v, or a".to_string())
    }
}