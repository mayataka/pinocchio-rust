use cxx::UniquePtr;
use super::model;

#[cxx::bridge(namespace = "pinocchio")]
pub mod ffi {
    unsafe extern "C++" {
        include!("multibody/data.hpp");

        pub type Data;
        type Model = model::ffi::Model;
        fn new_data(model: &UniquePtr<Model>) -> UniquePtr<Data>;
        fn clone(data : &Data) -> UniquePtr<Data>;
    }
}


pub mod ffi {
    pub unsafe impl Data {
        pub fn new(model: &UniquePtr<Model>) -> UniquePtr<Data> {
            new_data(model)
        }
        pub fn clone(&self) -> UniquePtr<Data> {
            clone(&self)
        }
    }
}