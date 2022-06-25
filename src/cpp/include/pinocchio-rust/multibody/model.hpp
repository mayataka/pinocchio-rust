#pragma once

#include "pinocchio/multibody/model.hpp"

#include <memory>

namespace pinocchio {

std::unique_ptr<Model> create_model();

std::unique_ptr<Model> clone_model(const std::unique_ptr<Model>& model);

void build_model_from_urdf(std::unique_ptr<Model>& model, const std::string& urdf_path,
                           const bool& floating_base);

}