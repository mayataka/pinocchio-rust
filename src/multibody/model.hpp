#pragma once

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/parsers/urdf.hpp"

#include <memory>

namespace pinocchio {

std::unique_ptr<Model> new_model();
std::unique_ptr<Model> clone(const Model& model);
std::unique_ptr<Model> clone(const std::unique_ptr<Model>& model);

// void buildFromUrdf(const std::unique_ptr<Model>& model);

// pinocchio::urdf::buildModel(path_to_urdf, 
//                             pinocchio::JointModelFreeFlyer(), model_);

}