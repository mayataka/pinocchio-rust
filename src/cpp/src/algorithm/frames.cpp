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

void getFrameVelocity(const std::unique_ptr<Model>& model,
                      const std::unique_ptr<Data>& data,
                      const std::uint32_t frame_id,
                      const std::uint32_t rf,
                      rust::Slice<double> vel) {
  Eigen::Vector6dMap(vel)
      = getFrameVelocity(*model.get(), *data.get(), frame_id, 
                         ReferenceFrameFromUint32(rf)).toVector(); 
}

void getFrameAcceleration(const std::unique_ptr<Model>& model,
                          const std::unique_ptr<Data>& data,
                          const std::uint32_t frame_id,
                          const std::uint32_t rf,
                          rust::Slice<double> acc) {
  Eigen::Vector6dMap(acc)
      = getFrameAcceleration(*model.get(), *data.get(), frame_id, 
                             ReferenceFrameFromUint32(rf)).toVector(); 
}

void getFrameClassicalAcceleration(const std::unique_ptr<Model>& model,
                                   const std::unique_ptr<Data>& data,
                                   const std::uint32_t frame_id,
                                   const std::uint32_t rf,
                                   rust::Slice<double> acc) {
  Eigen::Vector6dMap(acc)
      = getFrameClassicalAcceleration(*model.get(), *data.get(), frame_id, 
                                      ReferenceFrameFromUint32(rf)).toVector(); 
}

void getFrameJacobian(const std::unique_ptr<Model>& model,
                      std::unique_ptr<Data>& data,
                      const std::uint32_t frame_id, const std::uint32_t rf,
                      rust::Slice<double> J) {
  getFrameJacobian(*model.get(), *data.get(), frame_id, 
                   ReferenceFrameFromUint32(rf),  
                   Eigen::MatrixXdMap(J, 6, model->nv)); 
}

void computeFrameJacobian(const std::unique_ptr<Model>& model,
                          std::unique_ptr<Data>& data,
                          rust::Slice<const double> q, 
                          const std::uint32_t frame_id, const std::uint32_t rf,
                          rust::Slice<double> J) {
  computeFrameJacobian(*model.get(), *data.get(), 
                       Eigen::ConstVectorXdMap(q, model->nq), frame_id, 
                       ReferenceFrameFromUint32(rf),
                       Eigen::MatrixXdMap(J, 6, model->nv)); 
}

void getFrameJacobianTimeVariation(const std::unique_ptr<Model>& model,
                                   std::unique_ptr<Data>& data,
                                   const std::uint32_t frame_id, const std::uint32_t rf,
                                   rust::Slice<double> dJ) {
  getFrameJacobianTimeVariation(*model.get(), *data.get(), frame_id, 
                                ReferenceFrameFromUint32(rf),
                                Eigen::MatrixXdMap(dJ, 6, model->nv)); 
}

rust::Vec<std::uint32_t> frameJacobianSize(const std::unique_ptr<Model>& model) {
  return rust::Vec<std::uint32_t>({6, static_cast<std::uint32_t>(model->nv)});
}

rust::Vec<std::uint32_t> frameJacobianTimeVaryationSize(const std::unique_ptr<Model>& model) {
  return rust::Vec<std::uint32_t>({6, static_cast<std::uint32_t>(model->nv)});
}

}