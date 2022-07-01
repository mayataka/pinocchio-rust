use cxx::{self, UniquePtr};
use nalgebra as na;

#[cxx::bridge(namespace = "pinocchio")]
pub mod ffi_joint_force_vector {
    unsafe extern "C++" {
        include!("pinocchio-rust/container/joint_force_vector.hpp");

        type JointForceVector;
        fn createJointForceVector(njoints: u32) -> UniquePtr<JointForceVector>;
        fn cloneJointForceVector(f: &UniquePtr<JointForceVector>) -> UniquePtr<JointForceVector>;
        fn getSize(f: &UniquePtr<JointForceVector>) -> u32;
        fn setForce(f: &mut UniquePtr<JointForceVector>, joint_id: u32, force: &[f64]);
    }
}


pub struct JointForceVector {
    pub ptr: UniquePtr<ffi_joint_force_vector::JointForceVector>,
}

impl JointForceVector {
    pub fn new(njoints: usize) -> JointForceVector {
        JointForceVector { ptr: ffi_joint_force_vector::createJointForceVector(njoints as u32) }
    }

    pub fn clone(&self) -> JointForceVector {
        JointForceVector { ptr: ffi_joint_force_vector::cloneJointForceVector(&self.ptr) }
    }

    pub fn size(&self) -> usize {
        ffi_joint_force_vector::getSize(&self.ptr) as usize
    }

    pub fn set_force(&mut self, joint_id: usize, force: &na::Vector6<f64>) -> Result<(), String> {
        if joint_id < self.size() {
            let joint_id = joint_id as u32;
            let force = force.as_slice();
            ffi_joint_force_vector::setForce(&mut self.ptr, joint_id, force);
            Ok(())
        }
        else {
            Err("Invalid joint_id".to_string())
        }
    }

}