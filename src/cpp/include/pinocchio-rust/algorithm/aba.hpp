#pragma once

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"

#include "rust/cxx.h"

#include <memory>
#include <vector>

namespace pinocchio {

void aba(const std::unique_ptr<Model>& model, std::unique_ptr<Data>& data,
         const rust::Vec<double>& q, const rust::Vec<double>& v, 
         const rust::Vec<double>& tau);

}