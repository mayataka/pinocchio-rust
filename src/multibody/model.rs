use cxx::{UniquePtr, CxxString, let_cxx_string, ExternType, type_id};
use std::pin::Pin;

#[cxx::bridge(namespace = "pinocchio")]
pub mod ffi {
    unsafe extern "C++" {
        include!("pinocchio-rust/multibody/model.hpp");

        type Model;
        fn create_model() -> UniquePtr<Model>;
        fn clone_model(model: &UniquePtr<Model>) -> UniquePtr<Model>;
        fn build_model_from_urdf(model: &mut UniquePtr<Model>, urdf_path: &CxxString, floating_base: &bool);
    }
}


pub enum BaseJointType {
    FixedBase,
    FloatingBase,
}


pub struct Model {
    pub ptr: UniquePtr<ffi::Model>,
}


impl Model {
    pub fn new() -> Model {
        Model { ptr: ffi::create_model() }
    }

    pub fn clone(&self) -> Model {
        Model { ptr: ffi::clone_model(&self.ptr) }
    }

    pub fn build_model_from_urdf(&mut self, urdf_path: &str, base_joint_type: &BaseJointType) {
        let_cxx_string!(cxx_urdf_path = urdf_path);
        if let BaseJointType::FloatingBase = base_joint_type {
            let floating_base = true;
            ffi::build_model_from_urdf(&mut self.ptr, &cxx_urdf_path, &floating_base);
        }
        else {
            let floating_base = false;
            ffi::build_model_from_urdf(&mut self.ptr, &cxx_urdf_path, &floating_base);
        }
    }
}