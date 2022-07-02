#pragma once

#include "pinocchio/multibody/model.hpp"

#include "rust/cxx.h"

#include <memory>
#include <vector>

namespace pinocchio {

void integrate(const std::unique_ptr<Model>& model,
               rust::Slice<const double> q, rust::Slice<const double> v,
               rust::Slice<double> qout);

void interpolate(const std::unique_ptr<Model>& model,
                 rust::Slice<const double> q0, rust::Slice<const double> q1,
                 const double u, rust::Slice<double> qout);

void difference(const std::unique_ptr<Model>& model,
                rust::Slice<const double> q0, rust::Slice<const double> q1,
                rust::Slice<double> dvout);

void randomConfiguration(const std::unique_ptr<Model>& model,
                         rust::Slice<const double> lower_limits,
                         rust::Slice<const double> upper_limits,
                         rust::Slice<double> qout);

void neutral(const std::unique_ptr<Model>& model, rust::Slice<double> qout);

}