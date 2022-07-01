#include "pinocchio-rust/algorithm/rnea.hpp"
#include "pinocchio/algorithm/aba.hpp"

#include "pinocchio-rust/math/eigen.hpp"


namespace pinocchio {


void aba(const std::unique_ptr<Model>& model, std::unique_ptr<Data>& data,
         rust::Slice<const double> q, rust::Slice<const double> v, 
         rust::Slice<const double> tau) {
  aba(*model.get(), *data.get(), 
      Eigen::ConstVectorXdMap(q, model->nq),
      Eigen::ConstVectorXdMap(v, model->nv),
      Eigen::ConstVectorXdMap(tau, model->nv));
}

void abaWithExternalForces(const std::unique_ptr<Model>& model, std::unique_ptr<Data>& data,
                           rust::Slice<const double> q, rust::Slice<const double> v, 
                           rust::Slice<const double> tau,
                           const std::unique_ptr<JointForceVector>& f) {
  aba(*model.get(), *data.get(), 
      Eigen::ConstVectorXdMap(q, model->nq),
      Eigen::ConstVectorXdMap(v, model->nv),
      Eigen::ConstVectorXdMap(tau, model->nv), *f.get());
}

void computeMinverse(const std::unique_ptr<Model>& model, std::unique_ptr<Data>& data,
                     rust::Slice<const double> q) {
  computeMinverse(*model.get(), *data.get(), Eigen::ConstVectorXdMap(q, model->nq));
}

}