#pragma once

#include "Eigen/Core"

#include <vector>
#include <memory>


namespace Eigen {

inline std::unique_ptr<std::vector<double>> Vector3dToStdVec(const Vector3d& v) {
  auto vec = std::make_unique<std::vector<double>>();
  for (int i=0; i<3; ++i) {
    vec->push_back(v.coeff(i));
  }
  return vec;
}

inline std::unique_ptr<std::vector<double>> Vector4dToStdVec(const Vector4d& v) {
  auto vec = std::make_unique<std::vector<double>>();
  for (int i=0; i<4; ++i) {
    vec->push_back(v.coeff(i));
  }
  return vec;
}

inline std::unique_ptr<std::vector<double>> VectorXdToStdVec(const VectorXd& v) {
  auto vec = std::make_unique<std::vector<double>>();
  const auto size = v.size();
  for (int i=0; i<size; ++i) {
    vec->push_back(v.coeff(i));
  }
  return vec;
}

inline std::unique_ptr<std::vector<double>> Matrix3dToStdVec(const Matrix3d& m) {
  auto vec = std::make_unique<std::vector<double>>();
  for (int i=0; i<3; ++i) {
    for (int j=0; j<3; ++i) {
      vec->push_back(m.coeff(i, j));
    }
  }
  return vec;
}

inline std::unique_ptr<std::vector<double>> Matrix4dToStdVec(const Matrix4d& m) {
  auto vec = std::make_unique<std::vector<double>>();
  for (int i=0; i<4; ++i) {
    for (int j=0; j<4; ++i) {
      vec->push_back(m.coeff(i, j));
    }
  }
  return vec;
}

inline std::unique_ptr<std::vector<double>> MatrixXdToStdVec(const MatrixXd& m) {
  auto vec = std::make_unique<std::vector<double>>();
  const auto cols = m.cols();
  const auto rows = m.rows();
  for (int i=0; i<cols; ++i) {
    for (int j=0; j<rows; ++i) {
      vec->push_back(m.coeff(i, j));
    }
  }
  return vec;
}

inline Map<const Vector3d> StdVecToVector3d(const std::unique_ptr<std::vector<double>>& v) {
  assert(v->size() == 3);
  return Map<const Vector3d>(v->data());
}

inline Map<const Vector4d> StdVecToVector4d(const std::unique_ptr<std::vector<double>>& v) {
  assert(v->size() == 4);
  return Map<const Vector4d>(v->data());
}

inline Map<const VectorXd> StdVecToVectorXd(const std::unique_ptr<std::vector<double>>& v, 
                                            const int& size) {
  return Map<const VectorXd>(v->data(), size);
}

inline Map<const Matrix3d> StdVecToMatrix3d(const std::unique_ptr<std::vector<double>>& m) {
  return Map<const Matrix3d>(m->data());
}

inline Map<const Matrix4d> StdVecToMatrix4d(const std::unique_ptr<std::vector<double>>& m) {
  return Map<const Matrix4d>(m->data());
}

inline Map<const MatrixXd> StdVecToMatrixXd(const std::unique_ptr<std::vector<double>>& m,
                                            const int& cols, const int &rows) {
  return Map<const MatrixXd>(m->data(), cols, rows);
}

}