#include "pinocchio/multibody/model.hpp"

#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/parsers/sample-models.hpp"

#include <iostream>

namespace pinocchio {

std::unique_ptr<Model> createModel() {
  return std::make_unique<Model>();
}

std::unique_ptr<Model> cloneModel(const std::unique_ptr<Model>& model) {
  return std::make_unique<Model>(*model.get());
}

void buildModelFromUrdf(std::unique_ptr<Model>& model, 
                        const std::string& urdf_path,
                        const bool& floating_base) {
  if (floating_base) {
    urdf::buildModel(urdf_path, JointModelFreeFlyer(), *model.get());
  }
  else {
    urdf::buildModel(urdf_path, *model.get());
  }
}

void buildSampleManipulator(std::unique_ptr<Model>& model) {
  buildModels::manipulator(*model.get());
}

void buildSampleHumanoid(std::unique_ptr<Model>& model) {
  buildModels::humanoid(*model.get());
}

std::unique_ptr<std::string> display(const std::unique_ptr<Model>& model) {
  std::stringstream ss; 
  ss << *model.get();
  return std::make_unique<std::string>(ss.str());
}

}
