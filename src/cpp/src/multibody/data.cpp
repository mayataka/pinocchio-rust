#include "pinocchio-rust/multibody/data.hpp"

#include "pinocchio-rust/math/eigen.hpp"

namespace pinocchio {

std::unique_ptr<Data> createData(const std::unique_ptr<Model>& model) {
  return std::make_unique<Data>(*model.get());
}

std::unique_ptr<Data> cloneData(const std::unique_ptr<Data>& data) {
  return std::make_unique<Data>(*data.get());
}

std::uint32_t nframesInData(const std::unique_ptr<Data>& data) {
  return static_cast<std::uint32_t>(data->oMf.size());
}

std::uint32_t njointsInData(const std::unique_ptr<Data>& data) {
  return static_cast<std::uint32_t>(data->oMi.size());
}

std::unique_ptr<std::vector<double>> frameTranslation(const std::unique_ptr<Data>& data, 
                                                      const std::uint32_t& frame_id) {
  return Eigen::Vector3dToStdVecUniquePtr(data->oMf[frame_id].translation());
}

std::unique_ptr<std::vector<double>> frameRotation(const std::unique_ptr<Data>& data, 
                                                   const std::uint32_t& frame_id) {
  return Eigen::Matrix3dToStdVecUniquePtr(data->oMf[frame_id].rotation());
}

std::unique_ptr<std::vector<double>> jointTranslation(const std::unique_ptr<Data>& data,
                                                      const std::uint32_t& joint_id) {
  return Eigen::Vector3dToStdVecUniquePtr(data->oMi[joint_id].translation());
}

std::unique_ptr<std::vector<double>> jointRotation(const std::unique_ptr<Data>& data,
                                                   const std::uint32_t& joint_id) {
  return Eigen::Matrix3dToStdVecUniquePtr(data->oMi[joint_id].rotation());
}

std::unique_ptr<std::vector<double>> com(const std::unique_ptr<Data>& data) {
  return Eigen::Vector3dToStdVecUniquePtr(data->com[0]);
}

std::unique_ptr<std::vector<double>> vcom(const std::unique_ptr<Data>& data) {
  return Eigen::Vector3dToStdVecUniquePtr(data->vcom[0]);
}

std::unique_ptr<std::vector<double>> acom(const std::unique_ptr<Data>& data) {
  return Eigen::Vector3dToStdVecUniquePtr(data->acom[0]);
}

std::unique_ptr<std::vector<double>> J(const std::unique_ptr<Data>& data) {
  return Eigen::MatrixXdToStdVecUniquePtr(data->J);
}

std::unique_ptr<std::vector<std::uint32_t>> J_size(const std::unique_ptr<Data>& data) {
  return Eigen::MatrixSize(data->J);
}

std::unique_ptr<std::vector<double>> dJ(const std::unique_ptr<Data>& data) {
  return Eigen::MatrixXdToStdVecUniquePtr(data->dJ);
}

std::unique_ptr<std::vector<std::uint32_t>> dJ_size(const std::unique_ptr<Data>& data) {
  return Eigen::MatrixSize(data->dJ);
}

std::unique_ptr<std::vector<double>> M(const std::unique_ptr<Data>& data) {
  return Eigen::MatrixXdToStdVecUniquePtr(data->M);
}

std::unique_ptr<std::vector<std::uint32_t>> M_size(const std::unique_ptr<Data>& data) {
  return Eigen::MatrixSize(data->M);
}

std::unique_ptr<std::vector<double>> Minv(const std::unique_ptr<Data>& data) {
  return Eigen::MatrixXdToStdVecUniquePtr(data->Minv);
}

std::unique_ptr<std::vector<std::uint32_t>> Minv_size(const std::unique_ptr<Data>& data) {
  return Eigen::MatrixSize(data->Minv);
}

std::unique_ptr<std::vector<double>> ddq(const std::unique_ptr<Data>& data) {
  return Eigen::VectorXdToStdVecUniquePtr(data->ddq);
}

std::unique_ptr<std::vector<std::uint32_t>> ddq_size(const std::unique_ptr<Data>& data) {
  return Eigen::VectorSize(data->ddq);
}

std::unique_ptr<std::vector<double>> ddq_dq(const std::unique_ptr<Data>& data) {
  return Eigen::MatrixXdToStdVecUniquePtr(data->ddq_dq);
}

std::unique_ptr<std::vector<std::uint32_t>> ddq_dq_size(const std::unique_ptr<Data>& data) {
  return Eigen::MatrixSize(data->ddq_dq);
}

std::unique_ptr<std::vector<double>> ddq_dv(const std::unique_ptr<Data>& data) {
  return Eigen::MatrixXdToStdVecUniquePtr(data->ddq_dv);
}

std::unique_ptr<std::vector<std::uint32_t>> ddq_dv_size(const std::unique_ptr<Data>& data) {
  return Eigen::MatrixSize(data->ddq_dv);
}

std::unique_ptr<std::vector<double>> ddq_dtau(const std::unique_ptr<Data>& data) {
  return Eigen::MatrixXdToStdVecUniquePtr(data->Minv);
}

std::unique_ptr<std::vector<std::uint32_t>> ddq_dtau_size(const std::unique_ptr<Data>& data) {
  return Eigen::MatrixSize(data->Minv);
}

std::unique_ptr<std::vector<double>> tau(const std::unique_ptr<Data>& data) {
  return Eigen::VectorXdToStdVecUniquePtr(data->tau);
}

std::unique_ptr<std::vector<std::uint32_t>> tau_size(const std::unique_ptr<Data>& data) {
  return Eigen::VectorSize(data->tau);
}

std::unique_ptr<std::vector<double>> dtau_dq(const std::unique_ptr<Data>& data) {
  return Eigen::MatrixXdToStdVecUniquePtr(data->dtau_dq);
}

std::unique_ptr<std::vector<std::uint32_t>> dtau_dq_size(const std::unique_ptr<Data>& data) {
  return Eigen::MatrixSize(data->dtau_dq);
}

std::unique_ptr<std::vector<double>> dtau_dv(const std::unique_ptr<Data>& data) {
  return Eigen::MatrixXdToStdVecUniquePtr(data->dtau_dv);
}

std::unique_ptr<std::vector<std::uint32_t>> dtau_dv_size(const std::unique_ptr<Data>& data) {
  return Eigen::MatrixSize(data->dtau_dv);
}

std::unique_ptr<std::vector<double>> dtau_da(const std::unique_ptr<Data>& data) {
  return Eigen::MatrixXdToStdVecUniquePtr(data->M);
}

std::unique_ptr<std::vector<std::uint32_t>> dtau_da_size(const std::unique_ptr<Data>& data) {
  return Eigen::MatrixSize(data->M);
}

}