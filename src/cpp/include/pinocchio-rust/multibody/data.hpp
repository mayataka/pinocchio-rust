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

std::unique_ptr<std::vector<double>> frameTranslation(const std::unique_ptr<Data>& data, 
                                                      const std::uint32_t& frame_id);

std::unique_ptr<std::vector<double>> frameRotation(const std::unique_ptr<Data>& data, 
                                                   const std::uint32_t& frame_id);

std::unique_ptr<std::vector<double>> jointTranslation(const std::unique_ptr<Data>& data,
                                                      const std::uint32_t& joint_id);

std::unique_ptr<std::vector<double>> jointRotation(const std::unique_ptr<Data>& data,
                                                   const std::uint32_t& joint_id);

std::unique_ptr<std::vector<double>> com(const std::unique_ptr<Data>& data);

std::unique_ptr<std::vector<double>> vcom(const std::unique_ptr<Data>& data);

std::unique_ptr<std::vector<double>> acom(const std::unique_ptr<Data>& data);

std::unique_ptr<std::vector<double>> J(const std::unique_ptr<Data>& data);

std::unique_ptr<std::vector<std::uint32_t>> J_size(const std::unique_ptr<Data>& data);

std::unique_ptr<std::vector<double>> dJ(const std::unique_ptr<Data>& data);

std::unique_ptr<std::vector<std::uint32_t>> dJ_size(const std::unique_ptr<Data>& data);

std::unique_ptr<std::vector<double>> M(const std::unique_ptr<Data>& data);

std::unique_ptr<std::vector<std::uint32_t>> M_size(const std::unique_ptr<Data>& data);

std::unique_ptr<std::vector<double>> Minv(const std::unique_ptr<Data>& data);

std::unique_ptr<std::vector<std::uint32_t>> Minv_size(const std::unique_ptr<Data>& data);

std::unique_ptr<std::vector<double>> ddq(const std::unique_ptr<Data>& data);

std::unique_ptr<std::vector<std::uint32_t>> ddq_size(const std::unique_ptr<Data>& data);

std::unique_ptr<std::vector<double>> ddq_dq(const std::unique_ptr<Data>& data);

std::unique_ptr<std::vector<std::uint32_t>> ddq_dq_size(const std::unique_ptr<Data>& data);

std::unique_ptr<std::vector<double>> ddq_dv(const std::unique_ptr<Data>& data);

std::unique_ptr<std::vector<std::uint32_t>> ddq_dv_size(const std::unique_ptr<Data>& data);

std::unique_ptr<std::vector<double>> ddq_dtau(const std::unique_ptr<Data>& data);

std::unique_ptr<std::vector<std::uint32_t>> ddq_dtau_size(const std::unique_ptr<Data>& data);

std::unique_ptr<std::vector<double>> tau(const std::unique_ptr<Data>& data);

std::unique_ptr<std::vector<std::uint32_t>> tau_size(const std::unique_ptr<Data>& data);

std::unique_ptr<std::vector<double>> dtau_dq(const std::unique_ptr<Data>& data);

std::unique_ptr<std::vector<std::uint32_t>> dtau_dq_size(const std::unique_ptr<Data>& data);

std::unique_ptr<std::vector<double>> dtau_dv(const std::unique_ptr<Data>& data);

std::unique_ptr<std::vector<std::uint32_t>> dtau_dv_size(const std::unique_ptr<Data>& data);

std::unique_ptr<std::vector<double>> dtau_da(const std::unique_ptr<Data>& data);

std::unique_ptr<std::vector<std::uint32_t>> dtau_da_size(const std::unique_ptr<Data>& data);

}