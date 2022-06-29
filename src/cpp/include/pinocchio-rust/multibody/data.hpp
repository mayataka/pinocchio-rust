#pragma once

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"

#include "rust/cxx.h"

#include <memory>
#include <vector>

namespace pinocchio {

std::unique_ptr<Data> createData(const std::unique_ptr<Model>& model);

std::unique_ptr<Data> cloneData(const std::unique_ptr<Data>& data);

std::uint32_t nframesInData(const std::unique_ptr<Data>& data);

std::uint32_t njointsInData(const std::unique_ptr<Data>& data);

rust::Vec<double> frameTranslation(const std::unique_ptr<Data>& data, const std::uint32_t& frame_id);

rust::Vec<double> frameRotation(const std::unique_ptr<Data>& data, const std::uint32_t& frame_id);

rust::Vec<double> jointTranslation(const std::unique_ptr<Data>& data, const std::uint32_t& joint_id);

rust::Vec<double> jointRotation(const std::unique_ptr<Data>& data, const std::uint32_t& joint_id);

rust::Vec<double> com(const std::unique_ptr<Data>& data);

rust::Vec<double> vcom(const std::unique_ptr<Data>& data);

rust::Vec<double> acom(const std::unique_ptr<Data>& data);

rust::Vec<double> J(const std::unique_ptr<Data>& data);

rust::Vec<std::uint32_t> J_size(const std::unique_ptr<Data>& data);

rust::Vec<double> dJ(const std::unique_ptr<Data>& data);

rust::Vec<std::uint32_t> dJ_size(const std::unique_ptr<Data>& data);

rust::Vec<double> M(const std::unique_ptr<Data>& data);

rust::Vec<std::uint32_t> M_size(const std::unique_ptr<Data>& data);

rust::Vec<double> Minv(const std::unique_ptr<Data>& data);

rust::Vec<std::uint32_t> Minv_size(const std::unique_ptr<Data>& data);

rust::Vec<double> ddq(const std::unique_ptr<Data>& data);

rust::Vec<double> ddq_dq(const std::unique_ptr<Data>& data);

rust::Vec<std::uint32_t> ddq_dq_size(const std::unique_ptr<Data>& data);

rust::Vec<double> ddq_dv(const std::unique_ptr<Data>& data);

rust::Vec<std::uint32_t> ddq_dv_size(const std::unique_ptr<Data>& data);

rust::Vec<double> ddq_dtau(const std::unique_ptr<Data>& data);

rust::Vec<std::uint32_t> ddq_dtau_size(const std::unique_ptr<Data>& data);

rust::Vec<double> tau(const std::unique_ptr<Data>& data);

rust::Vec<double> dtau_dq(const std::unique_ptr<Data>& data);

rust::Vec<std::uint32_t> dtau_dq_size(const std::unique_ptr<Data>& data);

rust::Vec<double> dtau_dv(const std::unique_ptr<Data>& data);

rust::Vec<std::uint32_t> dtau_dv_size(const std::unique_ptr<Data>& data);

rust::Vec<double> dtau_da(const std::unique_ptr<Data>& data);

rust::Vec<std::uint32_t> dtau_da_size(const std::unique_ptr<Data>& data);

}