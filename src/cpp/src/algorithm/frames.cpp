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
                             const rust::Vec<double>& q) {
  framesForwardKinematics(*model.get(), *data.get(), 
                          Eigen::StdVecToVectorXdMap(q, model->nq));
}

}