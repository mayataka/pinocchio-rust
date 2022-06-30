#pragma once

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"

#include "rust/cxx.h"

#include <memory>
#include <vector>

namespace pinocchio {

void updateFramePlacements(const std::unique_ptr<Model>& model,
                           std::unique_ptr<Data>& data);

void framesForwardKinematics(const std::unique_ptr<Model>& model,
                             std::unique_ptr<Data>& data,
                             rust::Slice<const double> q);

// TODO: add reference frame 
rust::Vec<double> getFrameVelocity(const std::unique_ptr<Model>& model,
                                   const std::unique_ptr<Data>& data,
                                   const std::uint32_t frame_id);

// TODO: add reference frame 
rust::Vec<double> getFrameAcceleration(const std::unique_ptr<Model>& model,
                                       const std::unique_ptr<Data>& data,
                                       const std::uint32_t frame_id);

// TODO: add reference frame 
rust::Vec<double> getFrameClassicalAcceleration(const std::unique_ptr<Model>& model,
                                                const std::unique_ptr<Data>& data,
                                                const std::uint32_t frame_id);

// TODO: add reference frame 
rust::Vec<double> getFrameJacobian(const std::unique_ptr<Model>& model,
                                   std::unique_ptr<Data>& data,
                                   const std::uint32_t frame_id);

// TODO: add reference frame 
rust::Vec<double> computeFrameJacobian(const std::unique_ptr<Model>& model,
                                       std::unique_ptr<Data>& data,
                                       rust::Slice<const double> q, 
                                       const std::uint32_t frame_id);

// TODO: add reference frame 
rust::Vec<double> getFrameJacobianTimeVariation(const std::unique_ptr<Model>& model,
                                                std::unique_ptr<Data>& data,
                                                const std::uint32_t frame_id);
}