use cxx::UniquePtr;

#[cxx::bridge(namespace = "pinocchio")]
pub mod ffi {
    unsafe extern "C++" {
        include!("multibody/model.hpp");

        pub type Model;
        fn new_model() -> UniquePtr<Model>;
        fn clone(model: &Model) -> UniquePtr<Model>;
    }
}

pub mod ffi {
    pub unsafe impl Model {
        pub fn new() -> UniquePtr<Model> {
            return new_model()
        }
        pub fn clone(&self) -> UniquePtr<Model> {
            return clone(&self)
        }
    }
}