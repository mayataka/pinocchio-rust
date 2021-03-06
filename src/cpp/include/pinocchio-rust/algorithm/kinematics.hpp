#pragma once

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"

#include "rust/cxx.h"

#include <memory>
#include <vector>

namespace pinocchio {

void forwardKinematicsPositionLevel(const std::unique_ptr<Model>& model,
                                    std::unique_ptr<Data>& data,
                                    rust::Slice<const double> q);

void forwardKinematicsVelocityLevel(const std::unique_ptr<Model>& model,
                                    std::unique_ptr<Data>& data,
                                    rust::Slice<const double> q,
                                    rust::Slice<const double> v);

void forwardKinematicsAccelerationLevel(const std::unique_ptr<Model>& model,
                                        std::unique_ptr<Data>& data,
                                        rust::Slice<const double> q,
                                        rust::Slice<const double> v,
                                        rust::Slice<const double> a);

}