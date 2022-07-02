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

void frameTranslation(const std::unique_ptr<Data>& data, const std::uint32_t frame_id, rust::Slice<double> out);

void frameRotation(const std::unique_ptr<Data>& data, const std::uint32_t frame_id, rust::Slice<double> out);

void jointTranslation(const std::unique_ptr<Data>& data, const std::uint32_t joint_id, rust::Slice<double> out);

void jointRotation(const std::unique_ptr<Data>& data, const std::uint32_t joint_id, rust::Slice<double> out);

void com(const std::unique_ptr<Data>& data, rust::Slice<double> out);

void vcom(const std::unique_ptr<Data>& data, rust::Slice<double> out);

void acom(const std::unique_ptr<Data>& data, rust::Slice<double> out);

void J(const std::unique_ptr<Data>& data, rust::Slice<double> out);

rust::Vec<std::uint32_t> J_size(const std::unique_ptr<Data>& data);

void dJ(const std::unique_ptr<Data>& data, rust::Slice<double> out);

rust::Vec<std::uint32_t> dJ_size(const std::unique_ptr<Data>& data);

void M(const std::unique_ptr<Data>& data, rust::Slice<double> out);

rust::Vec<std::uint32_t> M_size(const std::unique_ptr<Data>& data);

void Minv(const std::unique_ptr<Data>& data, rust::Slice<double> out);

rust::Vec<std::uint32_t> Minv_size(const std::unique_ptr<Data>& data);

void ddq(const std::unique_ptr<Data>& data, rust::Slice<double> out);

std::uint32_t ddq_size(const std::unique_ptr<Data>& data);

void ddq_dq(const std::unique_ptr<Data>& data, rust::Slice<double> out);

rust::Vec<std::uint32_t> ddq_dq_size(const std::unique_ptr<Data>& data);

void ddq_dv(const std::unique_ptr<Data>& data, rust::Slice<double> out);

rust::Vec<std::uint32_t> ddq_dv_size(const std::unique_ptr<Data>& data);

void ddq_dtau(const std::unique_ptr<Data>& data, rust::Slice<double> out);

rust::Vec<std::uint32_t> ddq_dtau_size(const std::unique_ptr<Data>& data);

void tau(const std::unique_ptr<Data>& data, rust::Slice<double> out);

std::uint32_t tau_size(const std::unique_ptr<Data>& data);

void dtau_dq(const std::unique_ptr<Data>& data, rust::Slice<double> out);

rust::Vec<std::uint32_t> dtau_dq_size(const std::unique_ptr<Data>& data);

void dtau_dv(const std::unique_ptr<Data>& data, rust::Slice<double> out);

rust::Vec<std::uint32_t> dtau_dv_size(const std::unique_ptr<Data>& data);

void dtau_da(const std::unique_ptr<Data>& data, rust::Slice<double> out);

rust::Vec<std::uint32_t> dtau_da_size(const std::unique_ptr<Data>& data);

}