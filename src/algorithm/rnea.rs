use std::vec::Vec;
use crate::multibody::Model;
use crate::multibody::Data;

#[cxx::bridge(namespace = "pinocchio")]
pub mod ffi_rnea {
    unsafe extern "C++" {
        include!("pinocchio-rust/algorithm/rnea.hpp");

        type Model = crate::multibody::ffi_model::Model;
        type Data = crate::multibody::ffi_data::Data;
        fn rnea(model: &UniquePtr<Model>, data: &mut UniquePtr<Data>, 
                q: &Vec<f64>, v: &Vec<f64>, a: &Vec<f64>);
    }
}


pub fn rnea(model: &Model, data: &mut Data, 
            q: &Vec<f64>, v: &Vec<f64>, a: &Vec<f64>) {
    ffi_rnea::rnea(&model.ptr, &mut data.ptr, &q, &v, &a);
}