#include "pinocchio-rust/algorithm/joint_configuration.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"

#include "pinocchio-rust/math/eigen.hpp"

namespace pinocchio {

rust::Vec<double> integrate(const std::unique_ptr<Model>& model,
                            rust::Slice<const double> q,
                            rust::Slice<const double> v) {
  Eigen::VectorXd qout(model->nq);
  integrate(*model.get(), 
            Eigen::ConstVectorXdMap(q, model->nq), 
            Eigen::ConstVectorXdMap(v, model->nv), qout);
  return Eigen::VectorXdToRustVec(qout);
}


rust::Vec<double> interpolate(const std::unique_ptr<Model>& model,
                              rust::Slice<const double> q0,
                              rust::Slice<const double> q1,
                              const double u) {
  Eigen::VectorXd qout(model->nq);
  interpolate(*model.get(), 
              Eigen::ConstVectorXdMap(q0, model->nq), 
              Eigen::ConstVectorXdMap(q1, model->nq), 
              u, qout);
  return Eigen::VectorXdToRustVec(qout);
}

rust::Vec<double> difference(const std::unique_ptr<Model>& model,
                             rust::Slice<const double> q0,
                             rust::Slice<const double> q1) {
  Eigen::VectorXd dvout(model->nv);
  difference(*model.get(), 
             Eigen::ConstVectorXdMap(q0, model->nq), 
             Eigen::ConstVectorXdMap(q1, model->nq), 
             dvout);
  return Eigen::VectorXdToRustVec(dvout);
}

rust::Vec<double> randomConfiguration(const std::unique_ptr<Model>& model,
                                      rust::Slice<const double> lower_limits,
                                      rust::Slice<const double> upper_limits) {
  Eigen::VectorXd qout(model->nq);
  randomConfiguration(*model.get(), 
                      Eigen::ConstVectorXdMap(lower_limits, model->nq), 
                      Eigen::ConstVectorXdMap(upper_limits, model->nq), 
                      qout);
  return Eigen::VectorXdToRustVec(qout);
}

rust::Vec<double> neutral(const std::unique_ptr<Model>& model) {
  Eigen::VectorXd qout(model->nq);
  neutral(*model.get(), qout);
  return Eigen::VectorXdToRustVec(qout);
}

}