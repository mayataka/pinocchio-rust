use cxx::{UniquePtr, CxxString, let_cxx_string};
use std::fmt::{self, write};

#[cxx::bridge(namespace = "pinocchio")]
pub mod ffi_model {
    unsafe extern "C++" {
        include!("pinocchio-rust/multibody/model.hpp");

        type Model;
        fn createModel() -> UniquePtr<Model>;
        fn cloneModel(model: &UniquePtr<Model>) -> UniquePtr<Model>;
        fn buildModelFromUrdf(model: &mut UniquePtr<Model>, urdf_path: &CxxString, floating_base: &bool);
        fn buildSampleManipulator(model: &mut UniquePtr<Model>);
        fn buildSampleHumanoid(model: &mut UniquePtr<Model>);
        fn display(model: &UniquePtr<Model>) -> UniquePtr<CxxString>;
    }
}


pub enum BaseJointType {
    FixedBase,
    FloatingBase,
}


pub struct Model {
    pub ptr: UniquePtr<ffi_model::Model>,
}


impl Model {
    pub fn new() -> Model {
        Model { ptr: ffi_model::createModel() }
    }

    pub fn clone(&self) -> Model {
        Model { ptr: ffi_model::cloneModel(&self.ptr) }
    }

    pub fn build_model_from_urdf(&mut self, urdf_path: &str, base_joint_type: &BaseJointType) {
        let_cxx_string!(cxx_urdf_path = urdf_path);
        if let BaseJointType::FloatingBase = base_joint_type {
            let floating_base = true;
            ffi_model::buildModelFromUrdf(&mut self.ptr, &cxx_urdf_path, &floating_base);
        }
        else {
            let floating_base = false;
            ffi_model::buildModelFromUrdf(&mut self.ptr, &cxx_urdf_path, &floating_base);
        }
    }

    pub fn build_sample_manipulator(&mut self) {
        ffi_model::buildSampleManipulator(&mut self.ptr);
    }

    pub fn build_sample_humanoid(&mut self) {
        ffi_model::buildSampleHumanoid(&mut self.ptr);
    }
}


impl fmt::Display for Model {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        let cxx_str = ffi_model::display(&self.ptr);
        write!(f, "{}", cxx_str.to_string())
    }
}