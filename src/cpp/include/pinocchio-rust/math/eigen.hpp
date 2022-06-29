#pragma once

#include "Eigen/Core"

#include <vector>
#include <memory>


namespace Eigen {

template <typename Vector3dType>
inline std::unique_ptr<std::vector<double>> Vector3dToStdVecUniquePtr(const Vector3dType& v) {
  auto vec = std::make_unique<std::vector<double>>();
  vec->reserve(3);
  for (int i=0; i<3; ++i) {
    vec->push_back(v.coeff(i));
  }
  return vec;
}

template <typename Vector4dType>
inline std::unique_ptr<std::vector<double>> Vector4dToStdVecUniquePtr(const Vector4dType& v) {
  auto vec = std::make_unique<std::vector<double>>();
  vec->reserve(4);
  for (int i=0; i<4; ++i) {
    vec->push_back(v.coeff(i));
  }
  return vec;
}

template <typename VectorXdType>
inline std::unique_ptr<std::vector<double>> VectorXdToStdVecUniquePtr(const VectorXdType& v) {
  auto vec = std::make_unique<std::vector<double>>();
  const auto size = v.size();
  vec->reserve(size);
  for (int i=0; i<size; ++i) {
    vec->push_back(v.coeff(i));
  }
  return vec;
}

template <typename Matrix3dType>
inline std::unique_ptr<std::vector<double>> Matrix3dToStdVecUniquePtr(const Matrix3dType& m) {
  auto vec = std::make_unique<std::vector<double>>();
  vec->reserve(9);
  for (int i=0; i<3; ++i) {
    for (int j=0; j<3; ++j) {
      vec->push_back(m.coeff(i, j));
    }
  }
  return vec;
}

template <typename Matrix4dType>
inline std::unique_ptr<std::vector<double>> Matrix4dToStdVecUniquePtr(const Matrix4dType& m) {
  auto vec = std::make_unique<std::vector<double>>();
  vec->reserve(16);
  for (int i=0; i<4; ++i) {
    for (int j=0; j<4; ++j) {
      vec->push_back(m.coeff(i, j));
    }
  }
  return vec;
}

template <typename MatrixXdType>
inline std::unique_ptr<std::vector<double>> MatrixXdToStdVecUniquePtr(const MatrixXdType& m) {
  auto vec = std::make_unique<std::vector<double>>();
  const auto cols = m.cols();
  const auto rows = m.rows();
  vec->reserve(cols*rows);
  for (int i=0; i<cols; ++i) {
    for (int j=0; j<rows; ++j) {
      vec->push_back(m.coeff(i, j));
    }
  }
  return vec;
}

inline Map<const Vector3d> StdVecToVector3dMap(const std::unique_ptr<std::vector<double>>& v) {
  assert(v->size() == 3);
  return Map<const Vector3d>(v->data());
}

inline Map<const Vector4d> StdVecToVector4dMap(const std::unique_ptr<std::vector<double>>& v) {
  assert(v->size() == 4);
  return Map<const Vector4d>(v->data());
}

inline Map<const VectorXd> StdVecToVectorXdMap(const std::unique_ptr<std::vector<double>>& v, 
                                               const int& size) {
  return Map<const VectorXd>(v->data(), size);
}

inline Map<const Matrix3d> StdVecToMatrix3dMap(const std::unique_ptr<std::vector<double>>& m) {
  return Map<const Matrix3d>(m->data());
}

inline Map<const Matrix4d> StdVecToMatrix4dMap(const std::unique_ptr<std::vector<double>>& m) {
  return Map<const Matrix4d>(m->data());
}

inline Map<const MatrixXd> StdVecToMatrixXdMap(const std::unique_ptr<std::vector<double>>& m,
                                               const int& cols, const int &rows) {
  return Map<const MatrixXd>(m->data(), cols, rows);
}

}