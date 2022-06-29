use cxx::UniquePtr;
use nalgebra::{Vector3, Matrix3, DVector, DMatrix};
use crate::multibody::Model;
use crate::math::cxxvec;

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
        fn frameTranslation(data : &UniquePtr<Data>, frame_id: &u32) -> UniquePtr<CxxVector<f64>>;
        fn frameRotation(data : &UniquePtr<Data>, frame_id: &u32) -> UniquePtr<CxxVector<f64>>;
        fn jointTranslation(data : &UniquePtr<Data>, joint_id: &u32) -> UniquePtr<CxxVector<f64>>;
        fn jointRotation(data : &UniquePtr<Data>, joint_id: &u32) -> UniquePtr<CxxVector<f64>>;
        fn com(data : &UniquePtr<Data>) -> UniquePtr<CxxVector<f64>>;
        fn vcom(data : &UniquePtr<Data>) -> UniquePtr<CxxVector<f64>>;
        fn acom(data : &UniquePtr<Data>) -> UniquePtr<CxxVector<f64>>;
        fn J(data : &UniquePtr<Data>) -> UniquePtr<CxxVector<f64>>;
        fn J_size(data : &UniquePtr<Data>) -> UniquePtr<CxxVector<u32>>;
        fn dJ(data : &UniquePtr<Data>) -> UniquePtr<CxxVector<f64>>;
        fn dJ_size(data : &UniquePtr<Data>) -> UniquePtr<CxxVector<u32>>;
        fn M(data : &UniquePtr<Data>) -> UniquePtr<CxxVector<f64>>;
        fn M_size(data : &UniquePtr<Data>) -> UniquePtr<CxxVector<u32>>;
        fn Minv(data : &UniquePtr<Data>) -> UniquePtr<CxxVector<f64>>;
        fn Minv_size(data : &UniquePtr<Data>) -> UniquePtr<CxxVector<u32>>;
        fn ddq(data : &UniquePtr<Data>) -> UniquePtr<CxxVector<f64>>;
        fn ddq_size(data : &UniquePtr<Data>) -> UniquePtr<CxxVector<u32>>;
        fn ddq_dq(data : &UniquePtr<Data>) -> UniquePtr<CxxVector<f64>>;
        fn ddq_dq_size(data : &UniquePtr<Data>) -> UniquePtr<CxxVector<u32>>;
        fn ddq_dv(data : &UniquePtr<Data>) -> UniquePtr<CxxVector<f64>>;
        fn ddq_dv_size(data : &UniquePtr<Data>) -> UniquePtr<CxxVector<u32>>;
        fn ddq_dtau(data : &UniquePtr<Data>) -> UniquePtr<CxxVector<f64>>;
        fn ddq_dtau_size(data : &UniquePtr<Data>) -> UniquePtr<CxxVector<u32>>;
        fn tau(data : &UniquePtr<Data>) -> UniquePtr<CxxVector<f64>>;
        fn tau_size(data : &UniquePtr<Data>) -> UniquePtr<CxxVector<u32>>;
        fn dtau_dq(data : &UniquePtr<Data>) -> UniquePtr<CxxVector<f64>>;
        fn dtau_dq_size(data : &UniquePtr<Data>) -> UniquePtr<CxxVector<u32>>;
        fn dtau_dv(data : &UniquePtr<Data>) -> UniquePtr<CxxVector<f64>>;
        fn dtau_dv_size(data : &UniquePtr<Data>) -> UniquePtr<CxxVector<u32>>;
        fn dtau_da(data : &UniquePtr<Data>) -> UniquePtr<CxxVector<f64>>;
        fn dtau_da_size(data : &UniquePtr<Data>) -> UniquePtr<CxxVector<u32>>;
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

    pub fn nframes(&self) -> u32 {
        ffi_data::nframesInData(&self.ptr)
    }

    pub fn njoints(&self) -> u32 {
        ffi_data::njointsInData(&self.ptr)
    }

    pub fn frame_translation(&self, frame_id: u32) -> Option<Vector3<f64>> {
        if frame_id < self.nframes() {
            let trans = ffi_data::frameTranslation(&self.ptr, &frame_id);
            cxxvec::cxxvec_to_vector3(&trans)
        }
        else {
            None
        }
    }

    pub fn frame_rotation(&self, frame_id: u32) -> Option<Matrix3<f64>> {
        if frame_id < self.nframes() {
            let rot = ffi_data::frameRotation(&self.ptr, &frame_id);
            cxxvec::cxxvec_to_matrix3(&rot)
        }
        else {
            None
        }
    }

    pub fn joint_translation(&self, joint_id: u32) -> Option<Vector3<f64>> {
        if joint_id < self.njoints() {
            let trans = ffi_data::jointTranslation(&self.ptr, &joint_id);
            cxxvec::cxxvec_to_vector3(&trans)
        }
        else {
            None
        }
    }

    pub fn joint_rotation(&self, joint_id: u32) -> Option<Matrix3<f64>> {
        if joint_id < self.njoints() {
            let rot = ffi_data::jointRotation(&self.ptr, &joint_id);
            cxxvec::cxxvec_to_matrix3(&rot)
        }
        else {
            None
        }
    }

    pub fn com(&self) -> Option<Vector3<f64>> {
        let com = ffi_data::com(&self.ptr);
        cxxvec::cxxvec_to_vector3(&com)
    }

    pub fn vcom(&self) -> Option<Vector3<f64>> {
        let vcom = ffi_data::vcom(&self.ptr);
        cxxvec::cxxvec_to_vector3(&vcom)
    }

    pub fn acom(&self) -> Option<Vector3<f64>> {
        let acom = ffi_data::acom(&self.ptr);
        cxxvec::cxxvec_to_vector3(&acom)
    }

    pub fn J(&self) -> Option<DMatrix<f64>> {
        let J = ffi_data::J(&self.ptr);
        let size = ffi_data::J_size(&self.ptr);
        let rows = size.get(0).unwrap();
        let cols = size.get(1).unwrap();
        cxxvec::cxxvec_to_dmatrix(&J, *rows, *cols)
    }

    pub fn dJ(&self) -> Option<DMatrix<f64>> {
        let dJ = ffi_data::dJ(&self.ptr);
        let size = ffi_data::dJ_size(&self.ptr);
        let rows = size.get(0).unwrap();
        let cols = size.get(1).unwrap();
        cxxvec::cxxvec_to_dmatrix(&dJ, *rows, *cols)
    }

    pub fn M(&self) -> Option<DMatrix<f64>> {
        let M = ffi_data::M(&self.ptr);
        let size = ffi_data::M_size(&self.ptr);
        let rows = size.get(0).unwrap();
        let cols = size.get(1).unwrap();
        cxxvec::cxxvec_to_dmatrix(&M, *rows, *cols)
    }

    pub fn Minv(&self) -> Option<DMatrix<f64>> {
        let Minv = ffi_data::Minv(&self.ptr);
        let size = ffi_data::Minv_size(&self.ptr);
        let rows = size.get(0).unwrap();
        let cols = size.get(1).unwrap();
        cxxvec::cxxvec_to_dmatrix(&Minv, *rows, *cols)
    }

    pub fn ddq(&self) -> Option<DVector<f64>> {
        let ddq = ffi_data::ddq(&self.ptr);
        let size = ffi_data::ddq_size(&self.ptr);
        let size = size.get(0).unwrap();
        cxxvec::cxxvec_to_dvector(&ddq, *size)
    }

    pub fn ddq_dq(&self) -> Option<DMatrix<f64>> {
        let ddq_dq = ffi_data::ddq_dq(&self.ptr);
        let size = ffi_data::ddq_dq_size(&self.ptr);
        let rows = size.get(0).unwrap();
        let cols = size.get(1).unwrap();
        cxxvec::cxxvec_to_dmatrix(&ddq_dq, *rows, *cols)
    }

    pub fn ddq_dv(&self) -> Option<DMatrix<f64>> {
        let ddq_dv = ffi_data::ddq_dv(&self.ptr);
        let size = ffi_data::ddq_dv_size(&self.ptr);
        let rows = size.get(0).unwrap();
        let cols = size.get(1).unwrap();
        cxxvec::cxxvec_to_dmatrix(&ddq_dv, *rows, *cols)
    }

    pub fn ddq_dtau(&self) -> Option<DMatrix<f64>> {
        let ddq_dtau = ffi_data::ddq_dtau(&self.ptr);
        let size = ffi_data::ddq_dtau_size(&self.ptr);
        let rows = size.get(0).unwrap();
        let cols = size.get(1).unwrap();
        cxxvec::cxxvec_to_dmatrix(&ddq_dtau, *rows, *cols)
    }

    pub fn tau(&self) -> Option<DVector<f64>> {
        let tau = ffi_data::tau(&self.ptr);
        let size = ffi_data::tau_size(&self.ptr);
        let size = size.get(0).unwrap();
        cxxvec::cxxvec_to_dvector(&tau, *size)
    }

    pub fn dtau_dq(&self) -> Option<DMatrix<f64>> {
        let dtau_dq = ffi_data::dtau_dq(&self.ptr);
        let size = ffi_data::dtau_dq_size(&self.ptr);
        let rows = size.get(0).unwrap();
        let cols = size.get(1).unwrap();
        cxxvec::cxxvec_to_dmatrix(&dtau_dq, *rows, *cols)
    }

    pub fn dtau_dv(&self) -> Option<DMatrix<f64>> {
        let dtau_dv = ffi_data::dtau_dv(&self.ptr);
        let size = ffi_data::dtau_dv_size(&self.ptr);
        let rows = size.get(0).unwrap();
        let cols = size.get(1).unwrap();
        cxxvec::cxxvec_to_dmatrix(&dtau_dv, *rows, *cols)
    }

    pub fn dtau_da(&self) -> Option<DMatrix<f64>> {
        let dtau_da = ffi_data::dtau_da(&self.ptr);
        let size = ffi_data::dtau_da_size(&self.ptr);
        let rows = size.get(0).unwrap();
        let cols = size.get(1).unwrap();
        cxxvec::cxxvec_to_dmatrix(&dtau_da, *rows, *cols)
    }

}