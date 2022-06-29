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

rust::Vec<double> frameTranslation(const std::unique_ptr<Data>& data, const std::uint32_t& frame_id) {
  return Eigen::Vector3dToRustVec(data->oMf[frame_id].translation());
}

rust::Vec<double> frameRotation(const std::unique_ptr<Data>& data, const std::uint32_t& frame_id) {
  return Eigen::Matrix3dToRustVec(data->oMf[frame_id].rotation());
}

rust::Vec<double> jointTranslation(const std::unique_ptr<Data>& data, const std::uint32_t& joint_id) {
  return Eigen::Vector3dToRustVec(data->oMi[joint_id].translation());
}

rust::Vec<double> jointRotation(const std::unique_ptr<Data>& data, const std::uint32_t& joint_id) {
  return Eigen::Matrix3dToRustVec(data->oMi[joint_id].rotation());
}

rust::Vec<double> com(const std::unique_ptr<Data>& data) {
  return Eigen::Vector3dToRustVec(data->com[0]);
}

rust::Vec<double> vcom(const std::unique_ptr<Data>& data) {
  return Eigen::Vector3dToRustVec(data->vcom[0]);
}

rust::Vec<double> acom(const std::unique_ptr<Data>& data) {
  return Eigen::Vector3dToRustVec(data->acom[0]);
}

rust::Vec<double> J(const std::unique_ptr<Data>& data) {
  return Eigen::MatrixXdToRustVec(data->J);
}

rust::Vec<std::uint32_t> J_size(const std::unique_ptr<Data>& data) {
  return Eigen::MatrixSize(data->J);
}

rust::Vec<double> dJ(const std::unique_ptr<Data>& data) {
  return Eigen::MatrixXdToRustVec(data->dJ);
}

rust::Vec<std::uint32_t> dJ_size(const std::unique_ptr<Data>& data) {
  return Eigen::MatrixSize(data->dJ);
}

rust::Vec<double> M(const std::unique_ptr<Data>& data) {
  return Eigen::MatrixXdToRustVec(data->M);
}

rust::Vec<std::uint32_t> M_size(const std::unique_ptr<Data>& data) {
  return Eigen::MatrixSize(data->M);
}

rust::Vec<double> Minv(const std::unique_ptr<Data>& data) {
  return Eigen::MatrixXdToRustVec(data->Minv);
}

rust::Vec<std::uint32_t> Minv_size(const std::unique_ptr<Data>& data) {
  return Eigen::MatrixSize(data->Minv);
}

rust::Vec<double> ddq(const std::unique_ptr<Data>& data) {
  return Eigen::VectorXdToRustVec(data->ddq);
}

rust::Vec<double> ddq_dq(const std::unique_ptr<Data>& data) {
  return Eigen::MatrixXdToRustVec(data->ddq_dq);
}

rust::Vec<std::uint32_t> ddq_dq_size(const std::unique_ptr<Data>& data) {
  return Eigen::MatrixSize(data->ddq_dq);
}

rust::Vec<double> ddq_dv(const std::unique_ptr<Data>& data) {
  return Eigen::MatrixXdToRustVec(data->ddq_dv);
}

rust::Vec<std::uint32_t> ddq_dv_size(const std::unique_ptr<Data>& data) {
  return Eigen::MatrixSize(data->ddq_dv);
}

rust::Vec<double> ddq_dtau(const std::unique_ptr<Data>& data) {
  return Eigen::MatrixXdToRustVec(data->Minv);
}

rust::Vec<std::uint32_t> ddq_dtau_size(const std::unique_ptr<Data>& data) {
  return Eigen::MatrixSize(data->Minv);
}

rust::Vec<double> tau(const std::unique_ptr<Data>& data) {
  return Eigen::VectorXdToRustVec(data->tau);
}

rust::Vec<double> dtau_dq(const std::unique_ptr<Data>& data) {
  return Eigen::MatrixXdToRustVec(data->dtau_dq);
}

rust::Vec<std::uint32_t> dtau_dq_size(const std::unique_ptr<Data>& data) {
  return Eigen::MatrixSize(data->dtau_dq);
}

rust::Vec<double> dtau_dv(const std::unique_ptr<Data>& data) {
  return Eigen::MatrixXdToRustVec(data->dtau_dv);
}

rust::Vec<std::uint32_t> dtau_dv_size(const std::unique_ptr<Data>& data) {
  return Eigen::MatrixSize(data->dtau_dv);
}

rust::Vec<double> dtau_da(const std::unique_ptr<Data>& data) {
  return Eigen::MatrixXdToRustVec(data->M);
}

rust::Vec<std::uint32_t> dtau_da_size(const std::unique_ptr<Data>& data) {
  return Eigen::MatrixSize(data->M);
}

}