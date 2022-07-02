use std::vec::Vec;
use cxx::{self, UniquePtr};
use nalgebra as na;
use crate::multibody::Model;

#[cxx::bridge(namespace = "pinocchio")]
pub mod ffi_data {
    unsafe extern "C++" {
        include!("pinocchio-rust/multibody/data.hpp");

        type Model = crate::multibody::ffi_model::Model;
        type Data;
        fn createData(model: &UniquePtr<Model>) -> UniquePtr<Data>;
        fn cloneData(data: &UniquePtr<Data>) -> UniquePtr<Data>;
        fn nframesInData(data: &UniquePtr<Data>) -> u32;
        fn njointsInData(data: &UniquePtr<Data>) -> u32;
        fn frameTranslation(data : &UniquePtr<Data>, frame_id: u32, out: &mut [f64]);
        fn frameRotation(data : &UniquePtr<Data>, frame_id: u32, out: &mut [f64]);
        fn jointTranslation(data : &UniquePtr<Data>, joint_id: u32, out: &mut [f64]);
        fn jointRotation(data : &UniquePtr<Data>, joint_id: u32, out: &mut [f64]);
        fn com(data : &UniquePtr<Data>, out: &mut [f64]);
        fn vcom(data : &UniquePtr<Data>, out: &mut [f64]);
        fn acom(data : &UniquePtr<Data>, out: &mut [f64]);
        fn J(data : &UniquePtr<Data>, out: &mut [f64]);
        fn J_size(data : &UniquePtr<Data>) -> Vec<u32>;
        fn dJ(data : &UniquePtr<Data>, out: &mut [f64]);
        fn dJ_size(data : &UniquePtr<Data>) -> Vec<u32>;
        fn M(data : &UniquePtr<Data>, out: &mut [f64]);
        fn M_size(data : &UniquePtr<Data>) -> Vec<u32>;
        fn Minv(data : &UniquePtr<Data>, out: &mut [f64]);
        fn Minv_size(data : &UniquePtr<Data>) -> Vec<u32>;
        fn ddq(data : &UniquePtr<Data>, out: &mut [f64]);
        fn ddq_size(data : &UniquePtr<Data>) -> u32;
        fn ddq_dq(data : &UniquePtr<Data>, out: &mut [f64]);
        fn ddq_dq_size(data : &UniquePtr<Data>) -> Vec<u32>;
        fn ddq_dv(data : &UniquePtr<Data>, out: &mut [f64]);
        fn ddq_dv_size(data : &UniquePtr<Data>) -> Vec<u32>;
        fn ddq_dtau(data : &UniquePtr<Data>, out: &mut [f64]);
        fn ddq_dtau_size(data : &UniquePtr<Data>) -> Vec<u32>;
        fn tau(data : &UniquePtr<Data>, out: &mut [f64]);
        fn tau_size(data : &UniquePtr<Data>) -> u32;
        fn dtau_dq(data : &UniquePtr<Data>, out: &mut [f64]);
        fn dtau_dq_size(data : &UniquePtr<Data>) -> Vec<u32>;
        fn dtau_dv(data : &UniquePtr<Data>, out: &mut [f64]);
        fn dtau_dv_size(data : &UniquePtr<Data>) -> Vec<u32>;
        fn dtau_da(data : &UniquePtr<Data>, out: &mut [f64]);
        fn dtau_da_size(data : &UniquePtr<Data>) -> Vec<u32>;
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

    pub fn nframes(&self) -> usize {
        ffi_data::nframesInData(&self.ptr) as usize
    }

    pub fn njoints(&self) -> usize {
        ffi_data::njointsInData(&self.ptr) as usize
    }

    pub fn frame_translation(&self, frame_id: usize) -> Option<na::Vector3<f64>> {
        if frame_id < self.nframes() {
            let frame_id = frame_id as u32;
            let mut out = na::Vector3::<f64>::zeros();
            let out_mut_slice = out.as_mut_slice();
            ffi_data::frameTranslation(&self.ptr, frame_id, out_mut_slice);
            Some(out)
        }
        else {
            None
        }
    }

    pub fn frame_rotation(&self, frame_id: usize) -> Option<na::Matrix3<f64>> {
        if frame_id < self.nframes() {
            let frame_id = frame_id as u32;
            let mut out = na::Matrix3::<f64>::zeros();
            let out_mut_slice = out.as_mut_slice();
            ffi_data::frameRotation(&self.ptr, frame_id, out_mut_slice);
            Some(out)
        }
        else {
            None
        }
    }

    pub fn joint_translation(&self, joint_id: usize) -> Option<na::Vector3<f64>> {
        if joint_id < self.njoints() {
            let joint_id = joint_id as u32;
            let mut out = na::Vector3::<f64>::zeros();
            let out_mut_slice = out.as_mut_slice();
            ffi_data::jointTranslation(&self.ptr, joint_id, out_mut_slice);
            Some(out)
        }
        else {
            None
        }
    }

    pub fn joint_rotation(&self, joint_id: usize) -> Option<na::Matrix3<f64>> {
        if joint_id < self.njoints() {
            let joint_id = joint_id as u32;
            let mut out = na::Matrix3::<f64>::zeros();
            let out_mut_slice = out.as_mut_slice();
            ffi_data::jointRotation(&self.ptr, joint_id, out_mut_slice);
            Some(out)
        }
        else {
            None
        }
    }

    pub fn com(&self) -> na::Vector3<f64> {
        let mut out = na::Vector3::<f64>::zeros();
        let out_mut_slice = out.as_mut_slice();
        ffi_data::com(&self.ptr, out_mut_slice);
        out
    }

    pub fn vcom(&self) -> na::Vector3<f64> {
        let mut out = na::Vector3::<f64>::zeros();
        let out_mut_slice = out.as_mut_slice();
        ffi_data::vcom(&self.ptr, out_mut_slice);
        out
    }

    pub fn acom(&self) -> na::Vector3<f64> {
        let mut out = na::Vector3::<f64>::zeros();
        let out_mut_slice = out.as_mut_slice();
        ffi_data::acom(&self.ptr, out_mut_slice);
        out
    }

    pub fn J(&self) -> na::DMatrix<f64> {
        let size = ffi_data::J_size(&self.ptr);
        let rows = size[0] as usize;
        let cols = size[1] as usize;
        let mut out = na::DMatrix::<f64>::zeros(rows, cols);
        let out_mut_slice = out.as_mut_slice();
        ffi_data::J(&self.ptr, out_mut_slice);
        out
    }

    pub fn dJ(&self) -> na::DMatrix<f64> {
        let size = ffi_data::dJ_size(&self.ptr);
        let rows = size[0] as usize;
        let cols = size[1] as usize;
        let mut out = na::DMatrix::<f64>::zeros(rows, cols);
        let out_mut_slice = out.as_mut_slice();
        ffi_data::dJ(&self.ptr, out_mut_slice);
        out
    }

    pub fn M(&self) -> na::DMatrix<f64> {
        let size = ffi_data::M_size(&self.ptr);
        let rows = size[0] as usize;
        let cols = size[1] as usize;
        let mut out = na::DMatrix::<f64>::zeros(rows, cols);
        let out_mut_slice = out.as_mut_slice();
        ffi_data::M(&self.ptr, out_mut_slice);
        out
    }

    pub fn Minv(&self) -> na::DMatrix<f64> {
        let size = ffi_data::Minv_size(&self.ptr);
        let rows = size[0] as usize;
        let cols = size[1] as usize;
        let mut out = na::DMatrix::<f64>::zeros(rows, cols);
        let out_mut_slice = out.as_mut_slice();
        ffi_data::Minv(&self.ptr, out_mut_slice);
        out
    }

    pub fn ddq(&self) -> na::DVector<f64> {
        let size = ffi_data::ddq_size(&self.ptr) as usize;
        let mut out = na::DVector::<f64>::zeros(size);
        let out_mut_slice = out.as_mut_slice();
        ffi_data::ddq(&self.ptr, out_mut_slice);
        out 
    }

    pub fn ddq_dq(&self) -> na::DMatrix<f64> {
        let size = ffi_data::ddq_dq_size(&self.ptr);
        let rows = size[0] as usize;
        let cols = size[1] as usize;
        let mut out = na::DMatrix::<f64>::zeros(rows, cols);
        let out_mut_slice = out.as_mut_slice();
        ffi_data::ddq_dq(&self.ptr, out_mut_slice);
        out
    }

    pub fn ddq_dv(&self) -> na::DMatrix<f64> {
        let size = ffi_data::ddq_dv_size(&self.ptr);
        let rows = size[0] as usize;
        let cols = size[1] as usize;
        let mut out = na::DMatrix::<f64>::zeros(rows, cols);
        let out_mut_slice = out.as_mut_slice();
        ffi_data::ddq_dv(&self.ptr, out_mut_slice);
        out
    }

    pub fn ddq_dtau(&self) -> na::DMatrix<f64> {
        let size = ffi_data::ddq_dtau_size(&self.ptr);
        let rows = size[0] as usize;
        let cols = size[1] as usize;
        let mut out = na::DMatrix::<f64>::zeros(rows, cols);
        let out_mut_slice = out.as_mut_slice();
        ffi_data::ddq_dtau(&self.ptr, out_mut_slice);
        out
    }

    pub fn tau(&self) -> na::DVector<f64> {
        let size = ffi_data::tau_size(&self.ptr) as usize;
        let mut out = na::DVector::<f64>::zeros(size);
        let out_mut_slice = out.as_mut_slice();
        ffi_data::tau(&self.ptr, out_mut_slice);
        out 
    }

    pub fn dtau_dq(&self) -> na::DMatrix<f64> {
        let size = ffi_data::dtau_dq_size(&self.ptr);
        let rows = size[0] as usize;
        let cols = size[1] as usize;
        let mut out = na::DMatrix::<f64>::zeros(rows, cols);
        let out_mut_slice = out.as_mut_slice();
        ffi_data::dtau_dq(&self.ptr, out_mut_slice);
        out
    }

    pub fn dtau_dv(&self) -> na::DMatrix<f64> {
        let size = ffi_data::dtau_dv_size(&self.ptr);
        let rows = size[0] as usize;
        let cols = size[1] as usize;
        let mut out = na::DMatrix::<f64>::zeros(rows, cols);
        let out_mut_slice = out.as_mut_slice();
        ffi_data::dtau_dv(&self.ptr, out_mut_slice);
        out
    }

    pub fn dtau_da(&self) -> na::DMatrix<f64> {
        let size = ffi_data::dtau_da_size(&self.ptr);
        let rows = size[0] as usize;
        let cols = size[1] as usize;
        let mut out = na::DMatrix::<f64>::zeros(rows, cols);
        let out_mut_slice = out.as_mut_slice();
        ffi_data::dtau_da(&self.ptr, out_mut_slice);
        out
    }

}