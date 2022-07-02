#pragma once

#include "Eigen/Core"
#include "rust/cxx.h"

#include <memory>
#include <algorithm>


namespace Eigen {

using Vector6d = Matrix<double, 6, 1>;

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
inline Map<const Vector6d> ConstVector6dMap(const StdVecType& v) {
  assert(v.size() == 6);
  return Map<const Vector6d>(v.data());
}

template <typename StdVecType>
inline Map<Vector6d> Vector6dMap(StdVecType& v) {
  assert(v.size() == 6);
  return Map<Vector6d>(v.data());
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
inline Map<const MatrixXd> ConstMatrixXdMap(const StdVecType& m, const int rows, const int cols) {
  return Map<const MatrixXd>(m.data(), rows, cols);
}

template <typename StdVecType>
inline Map<MatrixXd> MatrixXdMap(StdVecType& m, const int rows, const int cols) {
  return Map<MatrixXd>(m.data(), rows, cols);
}

}