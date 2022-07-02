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

void frameTranslation(const std::unique_ptr<Data>& data, const std::uint32_t frame_id, rust::Slice<double> out) {
  Eigen::Vector3dMap(out) = data->oMf[frame_id].translation();
}

void frameRotation(const std::unique_ptr<Data>& data, const std::uint32_t frame_id, rust::Slice<double> out) {
  Eigen::Matrix3dMap(out) = data->oMf[frame_id].rotation();
}

void jointTranslation(const std::unique_ptr<Data>& data, const std::uint32_t joint_id, rust::Slice<double> out) {
  Eigen::Vector3dMap(out) = data->oMi[joint_id].translation();
}

void jointRotation(const std::unique_ptr<Data>& data, const std::uint32_t joint_id, rust::Slice<double> out) {
  Eigen::Matrix3dMap(out) = data->oMi[joint_id].rotation();
}

void com(const std::unique_ptr<Data>& data, rust::Slice<double> out) {
  Eigen::Vector3dMap(out) = data->com[0];
}

void vcom(const std::unique_ptr<Data>& data, rust::Slice<double> out) {
  Eigen::Vector3dMap(out) = data->vcom[0];
}

void acom(const std::unique_ptr<Data>& data, rust::Slice<double> out) {
  Eigen::Vector3dMap(out) = data->acom[0];
}

void J(const std::unique_ptr<Data>& data, rust::Slice<double> out) {
  Eigen::MatrixXdMap(out, data->J.rows(), data->J.cols()) = data->J;
}

rust::Vec<std::uint32_t> J_size(const std::unique_ptr<Data>& data) {
  return Eigen::MatrixSize(data->J);
}

void dJ(const std::unique_ptr<Data>& data, rust::Slice<double> out) {
  Eigen::MatrixXdMap(out, data->dJ.rows(), data->dJ.cols()) = data->dJ;
}

rust::Vec<std::uint32_t> dJ_size(const std::unique_ptr<Data>& data) {
  return Eigen::MatrixSize(data->dJ);
}

void M(const std::unique_ptr<Data>& data, rust::Slice<double> out) {
  Eigen::MatrixXdMap(out, data->M.rows(), data->M.cols()) = data->M;
}

rust::Vec<std::uint32_t> M_size(const std::unique_ptr<Data>& data) {
  return Eigen::MatrixSize(data->M);
}

void Minv(const std::unique_ptr<Data>& data, rust::Slice<double> out) {
  Eigen::MatrixXdMap(out, data->Minv.rows(), data->Minv.cols()) = data->Minv;
}

rust::Vec<std::uint32_t> Minv_size(const std::unique_ptr<Data>& data) {
  return Eigen::MatrixSize(data->Minv);
}

void ddq(const std::unique_ptr<Data>& data, rust::Slice<double> out) {
  Eigen::VectorXdMap(out, data->ddq.size()) = data->ddq;
}

std::uint32_t ddq_size(const std::unique_ptr<Data>& data) {
  return static_cast<std::uint32_t>(data->ddq.size());
}

void ddq_dq(const std::unique_ptr<Data>& data, rust::Slice<double> out) {
  Eigen::MatrixXdMap(out, data->ddq_dq.rows(), data->ddq_dq.cols()) = data->ddq_dq;
}

rust::Vec<std::uint32_t> ddq_dq_size(const std::unique_ptr<Data>& data) {
  return Eigen::MatrixSize(data->ddq_dq);
}

void ddq_dv(const std::unique_ptr<Data>& data, rust::Slice<double> out) {
  Eigen::MatrixXdMap(out, data->ddq_dv.rows(), data->ddq_dv.cols()) = data->ddq_dv;
}

rust::Vec<std::uint32_t> ddq_dv_size(const std::unique_ptr<Data>& data) {
  return Eigen::MatrixSize(data->ddq_dv);
}

void ddq_dtau(const std::unique_ptr<Data>& data, rust::Slice<double> out) {
  Eigen::MatrixXdMap(out, data->Minv.rows(), data->Minv.cols()) = data->Minv;
}

rust::Vec<std::uint32_t> ddq_dtau_size(const std::unique_ptr<Data>& data) {
  return Eigen::MatrixSize(data->Minv);
}

void tau(const std::unique_ptr<Data>& data, rust::Slice<double> out) {
  Eigen::VectorXdMap(out, data->tau.size()) = data->tau;
}

std::uint32_t tau_size(const std::unique_ptr<Data>& data) {
  return static_cast<std::uint32_t>(data->tau.size());
}

void dtau_dq(const std::unique_ptr<Data>& data, rust::Slice<double> out) {
  Eigen::MatrixXdMap(out, data->dtau_dq.rows(), data->dtau_dq.cols()) = data->dtau_dq;
}

rust::Vec<std::uint32_t> dtau_dq_size(const std::unique_ptr<Data>& data) {
  return Eigen::MatrixSize(data->dtau_dq);
}

void dtau_dv(const std::unique_ptr<Data>& data, rust::Slice<double> out) {
  Eigen::MatrixXdMap(out, data->dtau_dv.rows(), data->dtau_dv.cols()) = data->dtau_dv;
}

rust::Vec<std::uint32_t> dtau_dv_size(const std::unique_ptr<Data>& data) {
  return Eigen::MatrixSize(data->dtau_dv);
}

void dtau_da(const std::unique_ptr<Data>& data, rust::Slice<double> out) {
  Eigen::MatrixXdMap(out, data->M.rows(), data->M.cols()) = data->M;
}

rust::Vec<std::uint32_t> dtau_da_size(const std::unique_ptr<Data>& data) {
  return Eigen::MatrixSize(data->M);
}

}