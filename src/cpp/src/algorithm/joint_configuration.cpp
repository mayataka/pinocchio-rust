#include "pinocchio-rust/algorithm/joint_configuration.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"

#include "pinocchio-rust/math/eigen.hpp"

namespace pinocchio {

void integrate(const std::unique_ptr<Model>& model,
               rust::Slice<const double> q,
               rust::Slice<const double> v,
               rust::Slice<double> qout) {
  integrate(*model.get(), 
            Eigen::ConstVectorXdMap(q, model->nq), 
            Eigen::ConstVectorXdMap(v, model->nv), 
            Eigen::VectorXdMap(qout, model->nq));
}

void interpolate(const std::unique_ptr<Model>& model,
                 rust::Slice<const double> q0,
                 rust::Slice<const double> q1,
                 const double u,
                 rust::Slice<double> qout) {
  interpolate(*model.get(), 
              Eigen::ConstVectorXdMap(q0, model->nq), 
              Eigen::ConstVectorXdMap(q1, model->nq), 
              u, Eigen::VectorXdMap(qout, model->nq));
}

void difference(const std::unique_ptr<Model>& model,
                rust::Slice<const double> q0,
                rust::Slice<const double> q1,
                rust::Slice<double> dvout) {
  difference(*model.get(), 
             Eigen::ConstVectorXdMap(q0, model->nq), 
             Eigen::ConstVectorXdMap(q1, model->nq), 
             Eigen::VectorXdMap(dvout, model->nv));
}

void randomConfiguration(const std::unique_ptr<Model>& model,
                         rust::Slice<const double> lower_limits,
                         rust::Slice<const double> upper_limits,
                         rust::Slice<double> qout) {
  randomConfiguration(*model.get(), 
                      Eigen::ConstVectorXdMap(lower_limits, model->nq), 
                      Eigen::ConstVectorXdMap(upper_limits, model->nq), 
                      Eigen::VectorXdMap(qout, model->nq));
}

void neutral(const std::unique_ptr<Model>& model, rust::Slice<double> qout) {
  neutral(*model.get(), Eigen::VectorXdMap(qout, model->nq));
}

}