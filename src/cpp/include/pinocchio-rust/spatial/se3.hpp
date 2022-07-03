#pragma once

#include "pinocchio/spatial/se3.hpp"
#include "pinocchio/spatial/explog.hpp"

#include "rust/cxx.h"

#include <memory>

namespace pinocchio {

std::unique_ptr<SE3> createSE3();

std::unique_ptr<SE3> createSE3FromParts(rust::Slice<const double> rotation, 
                                        rust::Slice<const double> translation);

std::unique_ptr<SE3> cloneSE3(const std::unique_ptr<SE3>& se3);

void setRotation(std::unique_ptr<SE3>& se3, rust::Slice<const double> rotation);

void setTranslation(std::unique_ptr<SE3>& se3, rust::Slice<const double> translation);

void getRotation(const std::unique_ptr<SE3>& se3, rust::Slice<double> rotation_out);

void getTranslation(const std::unique_ptr<SE3>& se3, rust::Slice<double> translation_out);

void inverse(const std::unique_ptr<SE3>& se3, std::unique_ptr<SE3>& se3_inv_out);

void multiply(const std::unique_ptr<SE3>& left, const std::unique_ptr<SE3>& right, 
              std::unique_ptr<SE3>& out);

void log6(const std::unique_ptr<SE3>& se3, rust::Slice<double> log6out);

void Jlog6(const std::unique_ptr<SE3>& se3, rust::Slice<double> Jlog6out);

std::unique_ptr<std::string> displaySE3(const std::unique_ptr<SE3>& se3);

}