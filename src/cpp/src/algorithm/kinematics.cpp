#include "pinocchio-rust/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/kinematics.hpp"

#include "pinocchio-rust/math/eigen.hpp"


namespace pinocchio {

void forwardKinematicsPositionLevel(const std::unique_ptr<Model>& model,
                                    std::unique_ptr<Data>& data,
                                    rust::Slice<const double> q) {
  forwardKinematics(*model.get(), *data.get(), 
                    Eigen::ConstVectorXdMap(q, model->nq));
}

void forwardKinematicsVelocityLevel(const std::unique_ptr<Model>& model,
                                    std::unique_ptr<Data>& data,
                                    rust::Slice<const double> q,
                                    rust::Slice<const double> v) {
  forwardKinematics(*model.get(), *data.get(), 
                    Eigen::ConstVectorXdMap(q, model->nq),
                    Eigen::ConstVectorXdMap(v, model->nv));
}

void forwardKinematicsAccelerationLevel(const std::unique_ptr<Model>& model,
                                        std::unique_ptr<Data>& data,
                                        rust::Slice<const double> q,
                                        rust::Slice<const double> v,
                                        rust::Slice<const double> a) {
  forwardKinematics(*model.get(), *data.get(), 
                    Eigen::ConstVectorXdMap(q, model->nq),
                    Eigen::ConstVectorXdMap(v, model->nv),
                    Eigen::ConstVectorXdMap(a, model->nv));
}

}