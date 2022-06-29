use cxx::{UniquePtr, CxxVector};
use nalgebra::{Vector3, Matrix3, DVector, DMatrix};


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


pub fn cxxvec_to_dvector(cxx_vec: &UniquePtr<CxxVector<f64>>, size: u32) -> Option<DVector<f64>> {
    let size = size as usize;
    let len = cxx_vec.len();
    if len == size {
        let mut v = DVector::<f64>::zeros(size);
        for i in 0..size {
            v[i] = *cxx_vec.get(i).unwrap();
        }
        Some(v)
    }
    else {
        None
    }
}


pub unsafe fn cxxvec_to_dvector_unchecked(cxx_vec: &UniquePtr<CxxVector<f64>>, size: u32) -> Option<DVector<f64>> {
    let size = size as usize;
    let len = cxx_vec.len();
    if len == size {
        let mut v = DVector::<f64>::zeros(size);
        for i in 0..size {
            v[i] = *cxx_vec.get_unchecked(i);
        }
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


pub fn cxxvec_to_dmatrix(cxx_vec: &UniquePtr<CxxVector<f64>>, rows: u32, cols: u32) -> Option<DMatrix<f64>> {
    let rows = rows as usize;
    let cols = cols as usize;
    let len = cxx_vec.len();
    if len == (rows * cols) {
        let mut m = DMatrix::<f64>::zeros(rows, cols);
        for i in 0..rows {
            for j in 0..cols {
                m[(i, j)] = *cxx_vec.get(i*rows+j).unwrap();
            }
        }
        Some(m)
    }
    else {
        None
    }
}


pub unsafe fn cxxvec_to_dmatrix_unchecked(cxx_vec: &UniquePtr<CxxVector<f64>>, rows: u32, cols: u32) -> Option<DMatrix<f64>> {
    let rows = rows as usize;
    let cols = cols as usize;
    let len = cxx_vec.len();
    if len == (rows * cols) {
        let mut m = DMatrix::<f64>::zeros(rows, cols);
        for i in 0..rows {
            for j in 0..cols {
                m[(i, j)] = *cxx_vec.get_unchecked(i*rows+j);
            }
        }
        Some(m)
    }
    else {
        None
    }
}
