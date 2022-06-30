#pragma once

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"

#include "rust/cxx.h"

#include <memory>
#include <vector>

namespace pinocchio {

void aba(const std::unique_ptr<Model>& model, std::unique_ptr<Data>& data,
         rust::Slice<const double> q, rust::Slice<const double> v, 
         rust::Slice<const double> tau);

void computeMinverse(const std::unique_ptr<Model>& model, std::unique_ptr<Data>& data,
                     rust::Slice<const double> q);

}