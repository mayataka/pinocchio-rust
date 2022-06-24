#pragma once

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"

#include <memory>

namespace pinocchio {

std::unique_ptr<Data> new_data(const std::unique_ptr<Model>& model);
std::unique_ptr<Data> clone(const Data& data);

}