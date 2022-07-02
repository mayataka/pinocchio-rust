#pragma once

#include "pinocchio/multibody/model.hpp"

#include "rust/cxx.h"

#include <memory>

namespace pinocchio {

std::unique_ptr<Model> createModel();

std::unique_ptr<Model> cloneModel(const std::unique_ptr<Model>& model);

void buildModelFromUrdf(std::unique_ptr<Model>& model, const std::string& urdf_path,
                        const bool floating_base);

void buildSampleManipulator(std::unique_ptr<Model>& model);

void buildSampleHumanoid(std::unique_ptr<Model>& model);

bool existBodyName(const std::unique_ptr<Model>& model, const std::string &name);

bool existJointName(const std::unique_ptr<Model>& model, const std::string &name);

std::uint32_t getBodyId(const std::unique_ptr<Model>& model, const std::string &name);

std::uint32_t getJointId(const std::unique_ptr<Model>& model, const std::string &name);

std::unique_ptr<std::string> getModelName(const std::unique_ptr<Model>& model);

std::vector<std::unique_ptr<std::string>> getJointNames(const std::unique_ptr<Model>& model);

std::uint32_t nbodies(const std::unique_ptr<Model>& model);

std::uint32_t nframes(const std::unique_ptr<Model>& model);

std::uint32_t njoints(const std::unique_ptr<Model>& model);

std::uint32_t nq(const std::unique_ptr<Model>& model);

std::uint32_t nv(const std::unique_ptr<Model>& model);

void lowerPositionLimit(const std::unique_ptr<Model>& model,
                        rust::Slice<double> qout);

void upperPositionLimit(const std::unique_ptr<Model>& model,
                        rust::Slice<double> qout);

void velocityLimit(const std::unique_ptr<Model>& model,
                   rust::Slice<double> vout);

void effortLimit(const std::unique_ptr<Model>& model,
                 rust::Slice<double> tauout);

std::unique_ptr<std::string> display(const std::unique_ptr<Model>& model);

}