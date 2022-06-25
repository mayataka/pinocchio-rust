#pragma once

#include "pinocchio/multibody/model.hpp"

#include <memory>

namespace pinocchio {

std::unique_ptr<Model> createModel();

std::unique_ptr<Model> cloneModel(const std::unique_ptr<Model>& model);

void buildModelFromUrdf(std::unique_ptr<Model>& model, const std::string& urdf_path,
                        const bool& floating_base);

void buildSampleManipulator(std::unique_ptr<Model>& model);

void buildSampleHumanoid(std::unique_ptr<Model>& model);

std::unique_ptr<std::string> display(const std::unique_ptr<Model>& model);

}