#include "multibody/model.hpp"

namespace pinocchio {

std::unique_ptr<Model> new_model() {
  return std::make_unique<Model>();
}

std::unique_ptr<Model> clone(const Model& model) {
  return std::make_unique<Model>(model);
}


}