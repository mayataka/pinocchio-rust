use std::vec::Vec;
use crate::multibody::Model;
use crate::multibody::Data;

#[cxx::bridge(namespace = "pinocchio")]
pub mod ffi_aba {
    unsafe extern "C++" {
        include!("pinocchio-rust/algorithm/aba.hpp");

        type Model = crate::multibody::ffi_model::Model;
        type Data = crate::multibody::ffi_data::Data;
        fn aba(model: &UniquePtr<Model>, data: &mut UniquePtr<Data>, 
               q: &Vec<f64>, v: &Vec<f64>, tau: &Vec<f64>);
    }
}


pub fn aba(model: &Model, data: &mut Data, 
           q: &Vec<f64>, v: &Vec<f64>, tau: &Vec<f64>) {
    ffi_aba::aba(&model.ptr, &mut data.ptr, &q, &v, &tau);
}