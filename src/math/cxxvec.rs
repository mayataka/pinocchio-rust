use cxx::{UniquePtr, CxxVector};
use nalgebra::{Vector3, Matrix3};


pub fn cxxvec_to_vector3(cxx_vec: &UniquePtr<CxxVector<f64>>) -> Option<Vector3<f64>> {
    let size = cxx_vec.len();
    if size == 3 {
        let v0 = cxx_vec.get(0).unwrap();
        let v1 = cxx_vec.get(1).unwrap();
        let v2 = cxx_vec.get(2).unwrap();
        let v = Vector3::new(*v0, *v1, *v2);
        Some(v)
    }
    else {
        None
    }
}


pub unsafe fn cxxvec_to_vector3_unchecked(cxx_vec: &UniquePtr<CxxVector<f64>>) -> Option<Vector3<f64>> {
    let size = cxx_vec.len();
    if size == 3 {
        let v0 = cxx_vec.get_unchecked(0);
        let v1 = cxx_vec.get_unchecked(1);
        let v2 = cxx_vec.get_unchecked(2);
        let v = Vector3::new(*v0, *v1, *v2);
        Some(v)
    }
    else {
        None
    }
}


pub fn cxxvec_to_matrix3(cxx_vec: &UniquePtr<CxxVector<f64>>) -> Option<Matrix3<f64>> {
    let size = cxx_vec.len();
    if size == 9 {
        let v0 = cxx_vec.get(0).unwrap();
        let v1 = cxx_vec.get(1).unwrap();
        let v2 = cxx_vec.get(2).unwrap();
        let v3 = cxx_vec.get(3).unwrap();
        let v4 = cxx_vec.get(4).unwrap();
        let v5 = cxx_vec.get(5).unwrap();
        let v6 = cxx_vec.get(6).unwrap();
        let v7 = cxx_vec.get(7).unwrap();
        let v8 = cxx_vec.get(8).unwrap();
        let m = Matrix3::new(*v0, *v1, *v2, 
                                                                 *v3, *v4, *v5,
                                                                 *v6, *v7, *v8);
        Some(m)
    }
    else {
        None
    }
}


pub unsafe fn cxxvec_to_matrix3_unchecked(cxx_vec: &UniquePtr<CxxVector<f64>>) -> Option<Matrix3<f64>> {
    let size = cxx_vec.len();
    if size == 9 {
        let v0 = cxx_vec.get_unchecked(0);
        let v1 = cxx_vec.get_unchecked(1);
        let v2 = cxx_vec.get_unchecked(2);
        let v3 = cxx_vec.get_unchecked(3);
        let v4 = cxx_vec.get_unchecked(4);
        let v5 = cxx_vec.get_unchecked(5);
        let v6 = cxx_vec.get_unchecked(6);
        let v7 = cxx_vec.get_unchecked(7);
        let v8 = cxx_vec.get_unchecked(8);
        let m = Matrix3::new(*v0, *v1, *v2, 
                                                                 *v3, *v4, *v5,
                                                                 *v6, *v7, *v8);
        Some(m)
    }
    else {
        None
    }
}