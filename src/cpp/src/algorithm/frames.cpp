#include "pinocchio-rust/algorithm/frames.hpp"
#include "pinocchio/algorithm/frames.hpp"

#include "pinocchio-rust/math/eigen.hpp"


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
                                   const std::uint32_t frame_id) {
  return Eigen::Vector6dToRustVec(
      getFrameVelocity(*model.get(), *data.get(), frame_id, 
                       ReferenceFrame::LOCAL_WORLD_ALIGNED).toVector()); 
}

rust::Vec<double> getFrameAcceleration(const std::unique_ptr<Model>& model,
                                       const std::unique_ptr<Data>& data,
                                       const std::uint32_t frame_id) {
  return Eigen::Vector6dToRustVec(
      getFrameAcceleration(*model.get(), *data.get(), frame_id, 
                           ReferenceFrame::LOCAL_WORLD_ALIGNED).toVector()); 
}


rust::Vec<double> getFrameClassicalAcceleration(const std::unique_ptr<Model>& model,
                                                const std::unique_ptr<Data>& data,
                                                const std::uint32_t frame_id) {
  return Eigen::Vector6dToRustVec(
      getFrameClassicalAcceleration(*model.get(), *data.get(), frame_id, 
                                    ReferenceFrame::LOCAL_WORLD_ALIGNED).toVector()); 
}

}