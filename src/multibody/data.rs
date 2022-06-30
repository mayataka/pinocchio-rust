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
        fn frameTranslation(data : &UniquePtr<Data>, frame_id: &u32) -> Vec<f64>;
        fn frameRotation(data : &UniquePtr<Data>, frame_id: &u32) -> Vec<f64>;
        fn jointTranslation(data : &UniquePtr<Data>, joint_id: &u32) -> Vec<f64>;
        fn jointRotation(data : &UniquePtr<Data>, joint_id: &u32) -> Vec<f64>;
        fn com(data : &UniquePtr<Data>) -> Vec<f64>;
        fn vcom(data : &UniquePtr<Data>) -> Vec<f64>;
        fn acom(data : &UniquePtr<Data>) -> Vec<f64>;
        fn J(data : &UniquePtr<Data>) -> Vec<f64>;
        fn J_size(data : &UniquePtr<Data>) -> Vec<u32>;
        fn dJ(data : &UniquePtr<Data>) -> Vec<f64>;
        fn dJ_size(data : &UniquePtr<Data>) -> Vec<u32>;
        fn M(data : &UniquePtr<Data>) -> Vec<f64>;
        fn M_size(data : &UniquePtr<Data>) -> Vec<u32>;
        fn Minv(data : &UniquePtr<Data>) -> Vec<f64>;
        fn Minv_size(data : &UniquePtr<Data>) -> Vec<u32>;
        fn ddq(data : &UniquePtr<Data>) -> Vec<f64>;
        fn ddq_dq(data : &UniquePtr<Data>) -> Vec<f64>;
        fn ddq_dq_size(data : &UniquePtr<Data>) -> Vec<u32>;
        fn ddq_dv(data : &UniquePtr<Data>) -> Vec<f64>;
        fn ddq_dv_size(data : &UniquePtr<Data>) -> Vec<u32>;
        fn ddq_dtau(data : &UniquePtr<Data>) -> Vec<f64>;
        fn ddq_dtau_size(data : &UniquePtr<Data>) -> Vec<u32>;
        fn tau(data : &UniquePtr<Data>) -> Vec<f64>;
        fn dtau_dq(data : &UniquePtr<Data>) -> Vec<f64>;
        fn dtau_dq_size(data : &UniquePtr<Data>) -> Vec<u32>;
        fn dtau_dv(data : &UniquePtr<Data>) -> Vec<f64>;
        fn dtau_dv_size(data : &UniquePtr<Data>) -> Vec<u32>;
        fn dtau_da(data : &UniquePtr<Data>) -> Vec<f64>;
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
            let trans = ffi_data::frameTranslation(&self.ptr, &frame_id);
            Some(na::Vector3::from_vec(trans))
        }
        else {
            None
        }
    }

    pub fn frame_rotation(&self, frame_id: usize) -> Option<na::Matrix3<f64>> {
        if frame_id < self.nframes() {
            let frame_id = frame_id as u32;
            let rot = ffi_data::frameRotation(&self.ptr, &frame_id);
            Some(na::Matrix3::from_vec(rot))
        }
        else {
            None
        }
    }

    pub fn joint_translation(&self, joint_id: usize) -> Option<na::Vector3<f64>> {
        if joint_id < self.njoints() {
            let joint_id = joint_id as u32;
            let trans = ffi_data::jointTranslation(&self.ptr, &joint_id);
            Some(na::Vector3::from_vec(trans))
        }
        else {
            None
        }
    }

    pub fn joint_rotation(&self, joint_id: usize) -> Option<na::Matrix3<f64>> {
        if joint_id < self.njoints() {
            let joint_id = joint_id as u32;
            let rot = ffi_data::jointRotation(&self.ptr, &joint_id);
            Some(na::Matrix3::from_vec(rot))
        }
        else {
            None
        }
    }

    pub fn com(&self) -> Option<na::Vector3<f64>> {
        let com = ffi_data::com(&self.ptr);
        if com.len() == 3 {
            Some(na::Vector3::from_vec(com))
        }
        else {
            None
        }
    }

    pub fn vcom(&self) -> Option<na::Vector3<f64>> {
        let vcom = ffi_data::vcom(&self.ptr);
        if vcom.len() == 3 {
            Some(na::Vector3::from_vec(vcom))
        }
        else {
            None
        }
    }

    pub fn acom(&self) -> Option<na::Vector3<f64>> {
        let acom = ffi_data::acom(&self.ptr);
        if acom.len() == 3 {
            Some(na::Vector3::from_vec(acom))
        }
        else {
            None
        }
    }

    pub fn J(&self) -> Option<na::DMatrix<f64>> {
        let J = ffi_data::J(&self.ptr);
        let size = ffi_data::J_size(&self.ptr);
        let rows = size[0] as usize;
        let cols = size[1] as usize;
        Some(na::DMatrix::from_vec(rows, cols, J))
    }

    pub fn dJ(&self) -> Option<na::DMatrix<f64>> {
        let dJ = ffi_data::dJ(&self.ptr);
        let size = ffi_data::dJ_size(&self.ptr);
        let rows = size[0] as usize;
        let cols = size[1] as usize;
        Some(na::DMatrix::from_vec(rows, cols, dJ))
    }

    pub fn M(&self) -> Option<na::DMatrix<f64>> {
        let M = ffi_data::M(&self.ptr);
        let size = ffi_data::M_size(&self.ptr);
        let rows = size[0] as usize;
        let cols = size[1] as usize;
        Some(na::DMatrix::from_vec(rows, cols, M))
    }

    pub fn Minv(&self) -> Option<na::DMatrix<f64>> {
        let Minv = ffi_data::Minv(&self.ptr);
        let size = ffi_data::Minv_size(&self.ptr);
        let rows = size[0] as usize;
        let cols = size[1] as usize;
        Some(na::DMatrix::from_vec(rows, cols, Minv))
    }

    pub fn ddq(&self) -> Option<na::DVector<f64>> {
        let ddq = ffi_data::ddq(&self.ptr);
        Some(na::DVector::from_vec(ddq))
    }

    pub fn ddq_dq(&self) -> Option<na::DMatrix<f64>> {
        let ddq_dq = ffi_data::ddq_dq(&self.ptr);
        let size = ffi_data::ddq_dq_size(&self.ptr);
        let rows = size[0] as usize;
        let cols = size[1] as usize;
        Some(na::DMatrix::from_vec(rows, cols, ddq_dq))
    }

    pub fn ddq_dv(&self) -> Option<na::DMatrix<f64>> {
        let ddq_dv = ffi_data::ddq_dv(&self.ptr);
        let size = ffi_data::ddq_dv_size(&self.ptr);
        let rows = size[0] as usize;
        let cols = size[1] as usize;
        Some(na::DMatrix::from_vec(rows, cols, ddq_dv))
    }

    pub fn ddq_dtau(&self) -> Option<na::DMatrix<f64>> {
        let ddq_dtau = ffi_data::ddq_dtau(&self.ptr);
        let size = ffi_data::ddq_dtau_size(&self.ptr);
        let rows = size[0] as usize;
        let cols = size[1] as usize;
        Some(na::DMatrix::from_vec(rows, cols, ddq_dtau))
    }

    pub fn tau(&self) -> Option<na::DVector<f64>> {
        let tau = ffi_data::tau(&self.ptr);
        Some(na::DVector::from_vec(tau))
    }

    pub fn dtau_dq(&self) -> Option<na::DMatrix<f64>> {
        let dtau_dq = ffi_data::dtau_dq(&self.ptr);
        let size = ffi_data::dtau_dq_size(&self.ptr);
        let rows = size[0] as usize;
        let cols = size[1] as usize;
        Some(na::DMatrix::from_vec(rows, cols, dtau_dq))
    }

    pub fn dtau_dv(&self) -> Option<na::DMatrix<f64>> {
        let dtau_dv = ffi_data::dtau_dv(&self.ptr);
        let size = ffi_data::dtau_dv_size(&self.ptr);
        let rows = size[0] as usize;
        let cols = size[1] as usize;
        Some(na::DMatrix::from_vec(rows, cols, dtau_dv))
    }

    pub fn dtau_da(&self) -> Option<na::DMatrix<f64>> {
        let dtau_da = ffi_data::dtau_da(&self.ptr);
        let size = ffi_data::dtau_da_size(&self.ptr);
        let rows = size[0] as usize;
        let cols = size[1] as usize;
        Some(na::DMatrix::from_vec(rows, cols, dtau_da))
    }

}