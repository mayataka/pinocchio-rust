#pragma once

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"

#include <memory>
#include <vector>

namespace pinocchio {

std::unique_ptr<Data> createData(const std::unique_ptr<Model>& model);

std::unique_ptr<Data> cloneData(const std::unique_ptr<Data>& data);

std::uint32_t nframesInData(const std::unique_ptr<Data>& data);

std::uint32_t njointsInData(const std::unique_ptr<Data>& data);

// std::unique_ptr<std::vector<double>> frameTranslation(const std::unique_ptr<Data>& data, 
//                                                       const std::uint32_t& frame_id);

// std::unique_ptr<std::vector<double>> frameRotation(const std::unique_ptr<Data>& data, 
//                                                    const std::uint32_t& frame_id);

// std::unique_ptr<std::vector<double>> jointTranslation(const std::unique_ptr<Data>& data,
//                                                       const std::uint32_t& joint_id);

// std::unique_ptr<std::vector<double>> jointRotation(const std::unique_ptr<Data>& data,
//                                                    const std::uint32_t& joint_id);

}