use cxx;
use nalgebra as na;
use crate::multibody::Model;
use crate::multibody::Data;
use crate::container::JointForceVector;

#[cxx::bridge(namespace = "pinocchio")]
pub mod ffi_rnea {
    unsafe extern "C++" {
        include!("pinocchio-rust/algorithm/rnea.hpp");

        type Model = crate::multibody::ffi_model::Model;
        type Data = crate::multibody::ffi_data::Data;
        type JointForceVector = crate::container::ffi_joint_force_vector::JointForceVector;
        fn rnea(model: &UniquePtr<Model>, data: &mut UniquePtr<Data>, 
                q: &[f64], v: &[f64], a: &[f64]);
        fn rneaWithExternalForces(model: &UniquePtr<Model>, data: &mut UniquePtr<Data>, 
                                  q: &[f64], v: &[f64], a: &[f64], f: &UniquePtr<JointForceVector>);
    }
}


pub fn rnea(model: &Model, data: &mut Data, 
            q: &na::DVector<f64>, v: &na::DVector<f64>, a: &na::DVector<f64>) -> Result<(), String> {
    if q.len() == model.nq() && v.len() == model.nv() && a.len() == model.nv() {
        let q = q.as_slice();
        let v = v.as_slice();
        let a = a.as_slice();
        ffi_rnea::rnea(&model.ptr, &mut data.ptr, q, v, a);
        Ok(())
    }
    else {
        Err("Invalid sizes in q, v, or a".to_string())
    }
}

pub fn rnea_with_external_forces(model: &Model, data: &mut Data, 
                                 q: &na::DVector<f64>, v: &na::DVector<f64>, a: &na::DVector<f64>,
                                 f: &JointForceVector) -> Result<(), String> {
    if q.len() == model.nq() && v.len() == model.nv() && a.len() == model.nv() && f.size() == model.njoints() {
        let q = q.as_slice();
        let v = v.as_slice();
        let a = a.as_slice();
        ffi_rnea::rneaWithExternalForces(&model.ptr, &mut data.ptr, q, v, a, &f.ptr);
        Ok(())
    }
    else {
        Err("Invalid sizes in q, v, a, or f".to_string())
    }
}