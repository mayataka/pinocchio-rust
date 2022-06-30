use cxx;
use nalgebra as na;
use crate::multibody::Model;
use crate::multibody::Data;

#[cxx::bridge(namespace = "pinocchio")]
pub mod ffi_crba {
    unsafe extern "C++" {
        include!("pinocchio-rust/algorithm/crba.hpp");

        type Model = crate::multibody::ffi_model::Model;
        type Data = crate::multibody::ffi_data::Data;
        fn crba(model: &UniquePtr<Model>, data: &mut UniquePtr<Data>, q: &[f64]);
        fn crbaMinimal(model: &UniquePtr<Model>, data: &mut UniquePtr<Data>, q: &[f64]);
    }
}


pub fn crba(model: &Model, data: &mut Data, q: &na::DVector<f64>) -> Result<(), String> {
    if q.len() == model.nq() {
        let q = q.as_slice();
        ffi_crba::crba(&model.ptr, &mut data.ptr, q);
        Ok(())
    }
    else {
        Err("Invalid sizes in q".to_string())
    }
}


pub fn crba_minimal(model: &Model, data: &mut Data, q: &na::DVector<f64>) -> Result<(), String> {
    if q.len() == model.nq() {
        let q = q.as_slice();
        ffi_crba::crbaMinimal(&model.ptr, &mut data.ptr, q);
        Ok(())
    }
    else {
        Err("Invalid size in q".to_string())
    }
}