#pragma once

#include "pinocchio/container/aligned-vector.hpp"
#include "pinocchio/spatial/force.hpp"

#include "pinocchio-rust/multibody/model.hpp"

#include "rust/cxx.h"

#include <memory>
#include <vector>

namespace pinocchio {

using JointForceVector = container::aligned_vector<pinocchio::Force>;

std::unique_ptr<JointForceVector> createJointForceVector(const std::uint32_t njoints);

std::unique_ptr<JointForceVector> cloneJointForceVector(const std::unique_ptr<JointForceVector>& f);

std::uint32_t getSize(const std::unique_ptr<JointForceVector>& f);

void setForce(std::unique_ptr<JointForceVector>& f, 
              const std::uint32_t joint_id,
              rust::Slice<const double> force);

}