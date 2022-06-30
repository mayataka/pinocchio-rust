#pragma once

#include "Eigen/Core"
#include "rust/cxx.h"

#include <memory>
#include <algorithm>


namespace Eigen {

template <typename Vector3dType>
inline rust::Vec<double> Vector3dToRustVec(const Vector3dType& v) {
  return rust::Vec<double>({v.coeff(0), v.coeff(1), v.coeff(2)});
}

template <typename Vector4dType>
inline rust::Vec<double> Vector4dToRustVec(const Vector4dType& v) {
  return rust::Vec<double>({v.coeff(0), v.coeff(1), v.coeff(2), v.coeff(3)});
}

template <typename Vector6dType>
inline rust::Vec<double> Vector6dToRustVec(const Vector6dType& v) {
  return rust::Vec<double>({v.coeff(0), v.coeff(1), v.coeff(2), v.coeff(3), v.coeff(4), v.coeff(5)});
}

template <typename VectorXdType>
inline rust::Vec<double> VectorXdToRustVec(const VectorXdType& v) {
  const int size = v.size();
  auto vec = rust::Vec<double>();
  vec.reserve(size);
  for (int i=0; i<size; ++i) {
    vec.push_back(v.coeff(i));
  }
  return vec;
}

template <typename Matrix3dType>
inline rust::Vec<double> Matrix3dToRustVec(const Matrix3dType& m) {
  return rust::Vec<double>({m.coeff(0), m.coeff(1), m.coeff(2),
                            m.coeff(3), m.coeff(4), m.coeff(5), 
                            m.coeff(6), m.coeff(7), m.coeff(8)});
}

template <typename Matrix4dType>
inline rust::Vec<double> Matrix4dToRustVec(const Matrix4dType& m) {
  return rust::Vec<double>({m.coeff(0),  m.coeff(1),  m.coeff(2),  m.coeff(3),
                            m.coeff(4),  m.coeff(5),  m.coeff(6),  m.coeff(7),
                            m.coeff(8),  m.coeff(9),  m.coeff(10), m.coeff(11),
                            m.coeff(12), m.coeff(13), m.coeff(14), m.coeff(15)});
}

template <typename MatrixXdType>
inline rust::Vec<double> MatrixXdToRustVec(const MatrixXdType& m) {
  const int cols = m.cols();
  const int rows = m.rows();
  auto vec = rust::Vec<double>();
  vec.reserve(cols*rows);
  for (int i=0; i<cols; ++i) {
    for (int j=0; j<rows; ++j) {
      vec.push_back(m.coeff(i, j));
    }
  }
  return vec;
}

template <typename MatrixType>
inline rust::Vec<std::uint32_t> MatrixSize(const MatrixType& m) {
  return rust::Vec<std::uint32_t>({static_cast<std::uint32_t>(m.rows()),
                                   static_cast<std::uint32_t>(m.cols())});
}

template <typename StdVecType>
inline Map<const Vector3d> ConstVector3dMap(const StdVecType& v) {
  assert(v.size() == 3);
  return Map<const Vector3d>(v.data());
}

template <typename StdVecType>
inline Map<Vector3d> Vector3dMap(StdVecType& v) {
  assert(v.size() == 3);
  return Map<Vector3d>(v.data());
}

template <typename StdVecType>
inline Map<const VectorXd> ConstVectorXdMap(const StdVecType& v, const int& size) {
  return Map<const VectorXd>(v.data(), size);
}

template <typename StdVecType>
inline Map<VectorXd> VectorXdMap(StdVecType& v, const int& size) {
  return Map<VectorXd>(v.data(), size);
}

template <typename StdVecType>
inline Map<const Matrix3d> ConstMatrix3dMap(const StdVecType& m) {
  return Map<const Matrix3d>(m.data());
}

template <typename StdVecType>
inline Map<Matrix3d> Matrix3dMap(StdVecType& m) {
  return Map<Matrix3d>(m.data());
}

template <typename StdVecType>
inline Map<const MatrixXd> ConstMatrixXdMap(const StdVecType& m, const int& cols, const int &rows) {
  return Map<const MatrixXd>(m.data(), cols, rows);
}

template <typename StdVecType>
inline Map<MatrixXd> MatrixXdMap(StdVecType& m, const int& cols, const int &rows) {
  return Map<MatrixXd>(m.data(), cols, rows);
}

}