#pragma once

#include <Eigen/Core>
#include "pinocchio/spatial/se3.hpp"
#include "pinocchio/spatial/explog.hpp"

#include <memory>

#include "pinocchio-rust/math/eigen.hpp"

namespace pinocchio {

std::unique_ptr<SE3> createSE3();

std::unique_ptr<SE3> cloneSE3(const std::unique_ptr<SE3>& se3);

}