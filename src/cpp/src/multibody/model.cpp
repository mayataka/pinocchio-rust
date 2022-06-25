#include "pinocchio/multibody/model.hpp"

#include "pinocchio/parsers/urdf.hpp"

namespace pinocchio {

std::unique_ptr<Model> create_model() {
  return std::make_unique<Model>();
}

std::unique_ptr<Model> clone_model(const std::unique_ptr<Model>& model) {
  return std::make_unique<Model>(*model.get());
}

void build_model_from_urdf(std::unique_ptr<Model>& model, 
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
