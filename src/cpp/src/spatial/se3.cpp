#include "pinocchio-rust/spatial/se3.hpp"
#include "pinocchio-rust/math/eigen.hpp"


namespace pinocchio {

std::unique_ptr<SE3> createSE3() {
  return std::make_unique<SE3>();
}

std::unique_ptr<SE3> createSE3FromParts(rust::Slice<const double> rotation, 
                                        rust::Slice<const double> translation) {
  return std::make_unique<SE3>(Eigen::ConstMatrix3dMap(rotation), 
                               Eigen::ConstVector3dMap(translation));
}

std::unique_ptr<SE3> cloneSE3(const std::unique_ptr<SE3>& se3) {
  return std::make_unique<SE3>(*se3.get());
}

void setRotation(std::unique_ptr<SE3>& se3, rust::Slice<const double> rotation) {
  se3->rotation() = Eigen::ConstMatrix3dMap(rotation);
}

void setTranslation(std::unique_ptr<SE3>& se3, rust::Slice<const double> translation) {
  se3->translation() = Eigen::ConstVector3dMap(translation);
}

void getRotation(const std::unique_ptr<SE3>& se3, rust::Slice<double> rotation_out) {
  Eigen::Matrix3dMap(rotation_out) = se3->rotation();
}

void getTranslation(const std::unique_ptr<SE3>& se3, rust::Slice<double> translation_out) {
  Eigen::Vector3dMap(translation_out) = se3->translation();
}

void inverse(const std::unique_ptr<SE3>& se3, std::unique_ptr<SE3>& se3_inv_out) {
  *se3_inv_out.get() = se3->inverse();
}

void multiply(const std::unique_ptr<SE3>& left, const std::unique_ptr<SE3>& right, 
              std::unique_ptr<SE3>& out) {
  *out.get() = (*left.get()) * (*right.get());
}

void log6(const std::unique_ptr<SE3>& se3, rust::Slice<double> log6out) {
  Eigen::Vector6dMap(log6out) = log6(*se3.get()).toVector();
}

void Jlog6(const std::unique_ptr<SE3>& se3, rust::Slice<double> Jlog6out) {
  Jlog6(*se3.get(), Eigen::Matrix6dMap(Jlog6out));
}

std::unique_ptr<std::string> displaySE3(const std::unique_ptr<SE3>& se3) {
  std::stringstream ss; 
  ss << *se3.get();
  return std::make_unique<std::string>(ss.str());
}

}