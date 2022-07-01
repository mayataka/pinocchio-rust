#include "pinocchio-rust/algorithm/frames.hpp"
#include "pinocchio/algorithm/frames.hpp"

#include "pinocchio-rust/math/eigen.hpp"
#include "pinocchio-rust/algorithm/reference_frame.hpp"


namespace pinocchio {

void updateFramePlacements(const std::unique_ptr<Model>& model,
                           std::unique_ptr<Data>& data) {
  updateFramePlacements(*model.get(), *data.get());
}

void framesForwardKinematics(const std::unique_ptr<Model>& model,
                             std::unique_ptr<Data>& data,
                             rust::Slice<const double> q) {
  framesForwardKinematics(*model.get(), *data.get(), 
                          Eigen::ConstVectorXdMap(q, model->nq));
}

rust::Vec<double> getFrameVelocity(const std::unique_ptr<Model>& model,
                                   const std::unique_ptr<Data>& data,
                                   const std::uint32_t frame_id,
                                   const std::uint32_t rf) {
  return Eigen::Vector6dToRustVec(
      getFrameVelocity(*model.get(), *data.get(), frame_id, 
                       ReferenceFrameFromUint32(rf)).toVector()); 
}

rust::Vec<double> getFrameAcceleration(const std::unique_ptr<Model>& model,
                                       const std::unique_ptr<Data>& data,
                                       const std::uint32_t frame_id,
                                       const std::uint32_t rf) {
  return Eigen::Vector6dToRustVec(
      getFrameAcceleration(*model.get(), *data.get(), frame_id, 
                           ReferenceFrameFromUint32(rf)).toVector()); 
}

rust::Vec<double> getFrameClassicalAcceleration(const std::unique_ptr<Model>& model,
                                                const std::unique_ptr<Data>& data,
                                                const std::uint32_t frame_id,
                                                const std::uint32_t rf) {
  return Eigen::Vector6dToRustVec(
      getFrameClassicalAcceleration(*model.get(), *data.get(), frame_id, 
                                    ReferenceFrameFromUint32(rf)).toVector()); 
}

rust::Vec<double> getFrameJacobian(const std::unique_ptr<Model>& model,
                                   std::unique_ptr<Data>& data,
                                   const std::uint32_t frame_id,
                                   const std::uint32_t rf) {
  Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6, model->nv);
  getFrameJacobian(*model.get(), *data.get(), frame_id, 
                   ReferenceFrameFromUint32(rf), J); 
  return Eigen::MatrixXdToRustVec(J);
}

rust::Vec<double> computeFrameJacobian(const std::unique_ptr<Model>& model,
                                       std::unique_ptr<Data>& data,
                                       rust::Slice<const double> q, 
                                       const std::uint32_t frame_id,
                                       const std::uint32_t rf) {
  Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6, model->nv);
  computeFrameJacobian(*model.get(), *data.get(), 
                       Eigen::ConstVectorXdMap(q, model->nq), frame_id, 
                       ReferenceFrameFromUint32(rf), J); 
  return Eigen::MatrixXdToRustVec(J);
}

rust::Vec<double> getFrameJacobianTimeVariation(const std::unique_ptr<Model>& model,
                                                std::unique_ptr<Data>& data,
                                                const std::uint32_t frame_id,
                                                const std::uint32_t rf) {
  Eigen::MatrixXd dJ = Eigen::MatrixXd::Zero(6, model->nv);
  getFrameJacobianTimeVariation(*model.get(), *data.get(), frame_id, 
                                ReferenceFrameFromUint32(rf), dJ); 
  return Eigen::MatrixXdToRustVec(dJ);
}

rust::Vec<std::uint32_t> frameJacobianSize(const std::unique_ptr<Model>& model) {
  return rust::Vec<std::uint32_t>({6, model->nv});
}

rust::Vec<std::uint32_t> frameJacobianTimeVaryationSize(const std::unique_ptr<Model>& model) {
  return rust::Vec<std::uint32_t>({6, model->nv});
}

}