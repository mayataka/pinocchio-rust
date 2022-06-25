#include "pinocchio-rust/multibody/data.hpp"

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

}