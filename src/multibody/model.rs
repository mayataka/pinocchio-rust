use std::fmt::{self, write};
use cxx::{self, UniquePtr, CxxString, let_cxx_string};
use nalgebra as na;

#[cxx::bridge(namespace = "pinocchio")]
pub mod ffi_model {
    unsafe extern "C++" {
        include!("pinocchio-rust/multibody/model.hpp");

        type Model;
        fn createModel() -> UniquePtr<Model>;
        fn cloneModel(model: &UniquePtr<Model>) -> UniquePtr<Model>;
        fn buildModelFromUrdf(model: &mut UniquePtr<Model>, urdf_path: &CxxString, floating_base: bool);
        fn buildSampleManipulator(model: &mut UniquePtr<Model>);
        fn buildSampleHumanoid(model: &mut UniquePtr<Model>);
        fn existBodyName(model: &UniquePtr<Model>, name: &CxxString) -> bool;
        fn existJointName(model: &UniquePtr<Model>, name: &CxxString) -> bool;
        fn getBodyId(model: &UniquePtr<Model>, name: &CxxString) -> u32;
        fn getJointId(model: &UniquePtr<Model>, name: &CxxString) -> u32;
        fn getModelName(model: &UniquePtr<Model>) -> UniquePtr<CxxString>;
        fn nbodies(model: &UniquePtr<Model>) -> u32;
        fn nframes(model: &UniquePtr<Model>) -> u32;
        fn njoints(model: &UniquePtr<Model>) -> u32;
        fn nq(model: &UniquePtr<Model>) -> u32;
        fn nv(model: &UniquePtr<Model>) -> u32;
        fn lowerPositionLimit(data : &UniquePtr<Model>, qout: &mut [f64]);
        fn upperPositionLimit(data : &UniquePtr<Model>, qout: &mut [f64]);
        fn velocityLimit(data : &UniquePtr<Model>, vout: &mut [f64]);
        fn effortLimit(data : &UniquePtr<Model>, tauout: &mut [f64]);
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

    pub fn build_model_from_urdf(&mut self, urdf_path: &str, base_joint_type: BaseJointType) {
        let_cxx_string!(cxx_urdf_path = urdf_path);
        if let BaseJointType::FloatingBase = base_joint_type {
            let floating_base = true;
            ffi_model::buildModelFromUrdf(&mut self.ptr, &cxx_urdf_path, floating_base);
        }
        else {
            let floating_base = false;
            ffi_model::buildModelFromUrdf(&mut self.ptr, &cxx_urdf_path, floating_base);
        }
    }

    pub fn build_sample_manipulator(&mut self) {
        ffi_model::buildSampleManipulator(&mut self.ptr);
    }

    pub fn build_sample_humanoid(&mut self) {
        ffi_model::buildSampleHumanoid(&mut self.ptr);
    }

    pub fn frame_id(&self, name: &str) -> Option<usize> {
        let_cxx_string!(cxx_name = name);
        if ffi_model::existBodyName(&self.ptr, &cxx_name) {
            Some(ffi_model::getBodyId(&self.ptr, &cxx_name) as usize)
        }
        else {
            None
        }
    }

    pub fn joint_id(&self, name: &str) -> Option<usize> {
        let_cxx_string!(cxx_name = name);
        if ffi_model::existBodyName(&self.ptr, &cxx_name) {
            Some(ffi_model::getJointId(&self.ptr, &cxx_name) as usize)
        }
        else {
            None
        }
    }

    pub fn model_name(&self) -> String {
        ffi_model::getModelName(&self.ptr).to_string()
    }

    pub fn nbodies(&self) -> usize {
        ffi_model::nbodies(&self.ptr) as usize
    }

    pub fn nframes(&self) -> usize {
        ffi_model::nframes(&self.ptr) as usize
    }

    pub fn njoints(&self) -> usize {
        ffi_model::njoints(&self.ptr) as usize
    }

    pub fn nq(&self) -> usize {
        ffi_model::nq(&self.ptr) as usize
    }

    pub fn nv(&self) -> usize { 
        ffi_model::nv(&self.ptr) as usize
    }

    pub fn lower_position_limit(&self) -> na::DVector<f64> {
        let mut q = na::DVector::<f64>::zeros(self.nq());
        let q_mut_slice = q.as_mut_slice();
        ffi_model::lowerPositionLimit(&self.ptr, q_mut_slice);
        q
    }

    pub fn upper_position_limit(&self) -> na::DVector<f64> {
        let mut q = na::DVector::<f64>::zeros(self.nq());
        let q_mut_slice = q.as_mut_slice();
        ffi_model::upperPositionLimit(&self.ptr, q_mut_slice);
        q
    }

    pub fn velocity_limit(&self) -> na::DVector<f64> {
        let mut v = na::DVector::<f64>::zeros(self.nv());
        let v_mut_slice = v.as_mut_slice();
        ffi_model::velocityLimit(&self.ptr, v_mut_slice);
        v
    }

    pub fn effort_limit(&self) -> na::DVector<f64> {
        let mut tau = na::DVector::<f64>::zeros(self.nv());
        let tau_mut_slice = tau.as_mut_slice();
        ffi_model::effortLimit(&self.ptr, tau_mut_slice);
        tau
    }

}


impl fmt::Display for Model {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        let cxx_str = ffi_model::display(&self.ptr);
        write!(f, "{}", cxx_str.to_string())
    }
}