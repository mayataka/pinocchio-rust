use cxx;
use nalgebra as na;
use crate::multibody::Model;
use crate::multibody::Data;

#[cxx::bridge(namespace = "pinocchio")]
pub mod ffi_aba {
    unsafe extern "C++" {
        include!("pinocchio-rust/algorithm/aba.hpp");

        type Model = crate::multibody::ffi_model::Model;
        type Data = crate::multibody::ffi_data::Data;
        fn aba(model: &UniquePtr<Model>, data: &mut UniquePtr<Data>, 
               q: &[f64], v: &[f64], tau: &[f64]);
        fn computeMinverse(model: &UniquePtr<Model>, data: &mut UniquePtr<Data>, 
                           q: &[f64]);
    }
}


pub fn aba(model: &Model, data: &mut Data, 
           q: &na::DVector<f64>, v: &na::DVector<f64>, tau: &na::DVector<f64>) -> Result<(), String> {
    if q.len() == model.nq() && v.len() == model.nv() && tau.len() == model.nv() {
        let q = q.as_slice();
        let v = v.as_slice();
        let tau = tau.as_slice();
        ffi_aba::aba(&model.ptr, &mut data.ptr, q, v, tau);
        Ok(())
    }
    else {
        Err("Invalid sizes in q, v, or tau".to_string())
    }
}

pub fn compute_Minverse(model: &Model, data: &mut Data, 
                        q: &na::DVector<f64>) -> Result<(), String> {
    if q.len() == model.nq() {
        let q = q.as_slice();
        ffi_aba::computeMinverse(&model.ptr, &mut data.ptr, q);
        Ok(())
    }
    else {
        Err("Invalid sizes in q, v, or tau".to_string())
    }
}