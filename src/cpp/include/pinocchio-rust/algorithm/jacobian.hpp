#pragma once

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"

#include "rust/cxx.h"

#include <memory>
#include <vector>

namespace pinocchio {

void computeJointJacobians(const std::unique_ptr<Model>& model,
                           std::unique_ptr<Data>& data,
                           rust::Slice<const double> q);

void getJointJacobian(const std::unique_ptr<Model>& model,
                      std::unique_ptr<Data>& data, 
                      const std::uint32_t joint_id, const std::uint32_t rf, 
                      rust::Slice<double> J);

void computeJointJacobian(const std::unique_ptr<Model>& model,
                          std::unique_ptr<Data>& data,
                          rust::Slice<const double> q, 
                          const std::uint32_t joint_id, 
                          rust::Slice<double> J);

void computeJointJacobiansTimeVariation(const std::unique_ptr<Model>& model,
                                        std::unique_ptr<Data>& data,
                                        rust::Slice<const double> q,
                                        rust::Slice<const double> v);

void getJointJacobianTimeVariation(const std::unique_ptr<Model>& model,
                                   std::unique_ptr<Data>& data,
                                   const std::uint32_t joint_id, const std::uint32_t rf, 
                                   rust::Slice<double> dJ);

rust::Vec<std::uint32_t> jointJacobianSize(const std::unique_ptr<Model>& model);

rust::Vec<std::uint32_t> jointJacobianTimeVaryationSize(const std::unique_ptr<Model>& model);

}