#pragma once

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"

#include <memory>

namespace pinocchio {

std::unique_ptr<Data> createData(const std::unique_ptr<Model>& model);

std::unique_ptr<Data> cloneData(const std::unique_ptr<Data>& data);

}