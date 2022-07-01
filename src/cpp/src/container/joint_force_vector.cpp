#include "pinocchio-rust/container/joint_force_vector.hpp"
#include "pinocchio-rust/math/eigen.hpp"

namespace pinocchio {

std::unique_ptr<JointForceVector> createJointForceVector(const std::uint32_t njoints) {
  auto f = std::make_unique<JointForceVector>();
  for (std::uint32_t i=0; i<njoints; ++i) {
    f->push_back(Force::Zero());
  }
  return f;
}

std::unique_ptr<JointForceVector> cloneJointForceVector(const std::unique_ptr<JointForceVector>& f) {
  auto f_new = std::make_unique<JointForceVector>();
  for (int i=0; i<f->size(); ++i) {
    f_new->push_back((*f)[i]);
  }
  return f_new;
}

std::uint32_t getSize(const std::unique_ptr<JointForceVector>& f) {
  return static_cast<std::uint32_t>(f->size());
}

void setForce(std::unique_ptr<JointForceVector>& f, 
              const std::uint32_t joint_id,
              rust::Slice<const double> force) {
  (*f)[joint_id] = Force(Eigen::ConstVector6dMap(force));
}

}