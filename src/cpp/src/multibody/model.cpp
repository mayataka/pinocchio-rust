#include "pinocchio/multibody/model.hpp"

#include "pinocchio/parsers/urdf.hpp"

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
    pinocchio::urdf::buildModel(urdf_path, pinocchio::JointModelFreeFlyer(), *model.get());
  }
  else {
    pinocchio::urdf::buildModel(urdf_path, *model.get());
  }
}

}
