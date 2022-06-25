use cxx::UniquePtr;
use crate::multibody::Model;

#[cxx::bridge(namespace = "pinocchio")]
pub mod ffi_data {
    unsafe extern "C++" {
        include!("pinocchio-rust/multibody/data.hpp");

        type Model = crate::multibody::ffi_model::Model;
        type Data;
        fn createData(model: &UniquePtr<Model>) -> UniquePtr<Data>;
        fn cloneData(data : &UniquePtr<Data>) -> UniquePtr<Data>;
    }
}


pub struct Data {
    pub ptr: UniquePtr<ffi_data::Data>,
}


impl Data {
    pub fn new(model: &Model) -> Data {
        Data { ptr: ffi_data::createData(&model.ptr) }
    }

    pub fn clone(&self) -> Data {
        Data { ptr: ffi_data::cloneData(&self.ptr) }
    }
}