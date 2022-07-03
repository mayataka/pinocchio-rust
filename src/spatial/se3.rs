use std::fmt::{self, write};
use cxx::{self, UniquePtr, CxxString};
use nalgebra as na;

#[cxx::bridge(namespace = "pinocchio")]
pub mod ffi_se3 {
    unsafe extern "C++" {
        include!("pinocchio-rust/spatial/se3.hpp");

        type SE3;
        fn createSE3() -> UniquePtr<SE3>;
        fn createSE3FromParts(rotation: &[f64], translation: &[f64]) -> UniquePtr<SE3>;
        fn cloneSE3(se3: &UniquePtr<SE3>) -> UniquePtr<SE3>;
        fn setRotation(se3: &mut UniquePtr<SE3>, rotation: &[f64]);
        fn setTranslation(se3: &mut UniquePtr<SE3>, translation: &[f64]);
        fn getRotation(se3: &UniquePtr<SE3>, rotation: &mut [f64]);
        fn getTranslation(se3: &UniquePtr<SE3>, translation: &mut [f64]);
        fn inverse(se3: &UniquePtr<SE3>, se3_inv: &mut UniquePtr<SE3>);
        fn multiply(left: &UniquePtr<SE3>, right: &UniquePtr<SE3>, out: &mut UniquePtr<SE3>);
        fn log6(se3: &UniquePtr<SE3>, log6out: &mut [f64]);
        fn Jlog6(se3: &UniquePtr<SE3>, Jlog6out: &mut [f64]);
        fn displaySE3(se3: &UniquePtr<SE3>) -> UniquePtr<CxxString>;
    }
}


pub struct SE3 {
    pub ptr: UniquePtr<ffi_se3::SE3>,
}

impl SE3 {
    pub fn new() -> SE3 {
        SE3 { ptr: ffi_se3::createSE3() }
    }

    pub fn new_from_parts(rotation: &na::Matrix3::<f64>, translation: &na::Vector3::<f64>) -> SE3 {
        let rotation = rotation.as_slice();
        let translation = translation.as_slice();
        SE3 { ptr: ffi_se3::createSE3FromParts(rotation, translation) }
    }

    pub fn clone(&self) -> SE3 {
        SE3 { ptr: ffi_se3::cloneSE3(&self.ptr) }
    }

    pub fn set_rotation(&mut self, rotation: &na::Matrix3::<f64>) {
        let rotation = rotation.as_slice();
        ffi_se3::setRotation(&mut self.ptr, rotation);
    }

    pub fn set_translation(&mut self, translation: &na::Vector3::<f64>) {
        let translation = translation.as_slice();
        ffi_se3::setTranslation(&mut self.ptr, translation);
    }

    pub fn rotation(&self) -> na::Matrix3<f64> {
        let mut out = na::Matrix3::<f64>::zeros();
        let out_mut_slice = out.as_mut_slice();
        ffi_se3::getRotation(&self.ptr, out_mut_slice);
        out
    }

    pub fn translation(&self) -> na::Vector3<f64> {
        let mut out = na::Vector3::<f64>::zeros();
        let out_mut_slice = out.as_mut_slice();
        ffi_se3::getTranslation(&self.ptr, out_mut_slice);
        out
    }

    pub fn to_isometry3(&self) -> na::Isometry3<f64> {
        let rotation = self.rotation();
        let rotation = na::UnitQuaternion::<f64>::from_matrix(&rotation);
        let translation = self.translation();
        let translation = na::Translation3::<f64>::from(translation);
        na::Isometry3::<f64>::from_parts(translation, rotation)
    }

    pub fn inverse(&self) -> SE3 {
        let mut out = SE3::new();
        ffi_se3::inverse(&self.ptr, &mut out.ptr);
        out
    }

    pub fn multiply(&self, other: &SE3) -> SE3 {
        let mut out = SE3::new();
        ffi_se3::multiply(&self.ptr, &other.ptr, &mut out.ptr);
        out
    }

    pub fn log6(&self) -> na::Vector6<f64> {
        let mut out = na::Vector6::<f64>::zeros();
        let out_mut_slice = out.as_mut_slice();
        ffi_se3::log6(&self.ptr, out_mut_slice);
        out
    }

    pub fn Jlog6(&self) -> na::Matrix6<f64> {
        let mut out = na::Matrix6::<f64>::zeros();
        let out_mut_slice = out.as_mut_slice();
        ffi_se3::Jlog6(&self.ptr, out_mut_slice);
        out
    }

}

impl fmt::Display for SE3 {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        let cxx_str = ffi_se3::displaySE3(&self.ptr);
        write!(f, "{}", cxx_str.to_string())
    }
}