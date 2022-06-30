use cxx;
use na::{Dynamic, Matrix};
use crate::multibody::Model;
use crate::multibody::Data;
use crate::algorithm::KinematicsLevel;

#[cxx::bridge(namespace = "pinocchio")]
pub mod ffi_kinematics {
    unsafe extern "C++" {
        include!("pinocchio-rust/algorithm/kinematics.hpp");

        type Model = crate::multibody::ffi_model::Model;
        type Data = crate::multibody::ffi_data::Model;
        fn forwardKinematicsPositionLevel(model: &UniquePtr<Model>, data: &mut UniquePtr<Data>, 
                                          q: &[f64]);
        fn forwardKinematicsVelocityLevel(model: &UniquePtr<Model>, data: &mut UniquePtr<Data>, 
                                          q: &[f64], v: &[f64]);
        fn forwardKinematicsAccelerationLevel(model: &UniquePtr<Model>, data: &mut UniquePtr<Data>, 
                                              q: &[f64], v: &[f64], a: &[f64]);
    }
}

pub fn forward_kinematics_position_level(model: &Model, data: &mut Data, q: &DVector) {
    ffi_kinematics::forwardKinematicsPositionLevel(&model.ptr, &mut data.ptr, &q);
}
