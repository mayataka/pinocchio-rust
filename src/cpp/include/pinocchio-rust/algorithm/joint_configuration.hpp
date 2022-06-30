#pragma once

#include "pinocchio/multibody/model.hpp"

#include "rust/cxx.h"

#include <memory>
#include <vector>

namespace pinocchio {

rust::Vec<double> integrate(const std::unique_ptr<Model>& model,
                            rust::Slice<const double> q,
                            rust::Slice<const double> v);

rust::Vec<double> interpolate(const std::unique_ptr<Model>& model,
                              rust::Slice<const double> q0,
                              rust::Slice<const double> q1,
                              const double u);

rust::Vec<double> difference(const std::unique_ptr<Model>& model,
                             rust::Slice<const double> q0,
                             rust::Slice<const double> q1);

rust::Vec<double> randomConfiguration(const std::unique_ptr<Model>& model,
                                      rust::Slice<const double> lower_limits,
                                      rust::Slice<const double> upper_limits);

rust::Vec<double> neutral(const std::unique_ptr<Model>& model);

}