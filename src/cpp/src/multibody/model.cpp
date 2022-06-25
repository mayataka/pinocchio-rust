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

bool existBodyName(const std::unique_ptr<Model>& model, const std::string &name) {
  return model->existBodyName(name);
}

bool existJointName(const std::unique_ptr<Model>& model, const std::string &name) {
  return model->existJointName(name);
}

std::int32_t getBodyId(const std::unique_ptr<Model>& model, const std::string &name) {
  return static_cast<std::int32_t>(model->getBodyId(name));
}

std::int32_t getJointId(const std::unique_ptr<Model>& model, const std::string &name) {
  return static_cast<std::int32_t>(model->getJointId(name));
}

std::unique_ptr<std::string> getModelName(const std::unique_ptr<Model>& model) {
  return std::make_unique<std::string>(model->name);
}

std::vector<std::unique_ptr<std::string>> getJointNames(const std::unique_ptr<Model>& model) {
  std::vector<std::unique_ptr<std::string>> ret;
  for (const auto& e : model->names) {
    ret.push_back(std::make_unique<std::string>(e));
  }
  return ret;
}

std::uint32_t nbodies(const std::unique_ptr<Model>& model) {
  return static_cast<std::uint32_t>(model->nbodies);
}

std::uint32_t nframes(const std::unique_ptr<Model>& model) {
  return static_cast<std::uint32_t>(model->nframes);
}

std::uint32_t njoints(const std::unique_ptr<Model>& model) {
  return static_cast<std::uint32_t>(model->njoints);
}

std::uint32_t nq(const std::unique_ptr<Model>& model) {
  return static_cast<std::uint32_t>(model->nq);
}

std::uint32_t nv(const std::unique_ptr<Model>& model) {
  return static_cast<std::uint32_t>(model->nv);
}

std::unique_ptr<std::string> display(const std::unique_ptr<Model>& model) {
  std::stringstream ss; 
  ss << *model.get();
  return std::make_unique<std::string>(ss.str());
}

}
