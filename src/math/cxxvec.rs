use cxx::UniquePtr;
use nalgebra::geometry::{Rotation3, Translation3, Isometry3};
use nalgebra::base::{Vector3, Matrix3};


pub fn cxxvec_to_vector3(cxx_vec: &UniquePtr<CxxVector<f64>>) -> Option<Vector3> {
    if cxx_vec.len() == 3 {
        Some(Vector3::new(cxx_vec.get_unchecked(0), cxx_vec.get_unchecked(1), cxx_vec.get_unchecked(2))) 
    }
    else {
        None
    }
}

pub fn cxxvec_to_matrix3(cxx_vec: &UniquePtr<CxxVector<f64>>) -> Option<Matrix3> {
    if cxx_vec.len() == 9 {
        Some(Matrix3::new(cxx_vec.get_unchecked(0), cxx_vec.get_unchecked(1), cxx_vec.get_unchecked(2),
                          cxx_vec.get_unchecked(3), cxx_vec.get_unchecked(4), cxx_vec.get_unchecked(5),
                          cxx_vec.get_unchecked(6), cxx_vec.get_unchecked(7), cxx_vec.get_unchecked(8))) 
    }
    else {
        None
    }
}