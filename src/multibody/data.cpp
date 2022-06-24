#include "multibody/data.hpp"

namespace pinocchio {

std::unique_ptr<Data> new_data(const std::unique_ptr<Model>& model) {
  return std::make_unique<Data>(*model.get());
}

std::unique_ptr<Data> clone(const Data& data) {
  return std::make_unique<Data>(data);
}

}