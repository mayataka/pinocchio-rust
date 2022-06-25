#include "pinocchio-rust/multibody/data.hpp"

namespace pinocchio {

std::unique_ptr<Data> createData(const std::unique_ptr<Model>& model) {
  return std::make_unique<Data>(*model.get());
}

std::unique_ptr<Data> cloneData(const std::unique_ptr<Data>& data) {
  return std::make_unique<Data>(*data.get());
}

}