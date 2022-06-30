#include "pinocchio-rust/algorithm/rnea.hpp"
#include "pinocchio/algorithm/rnea.hpp"

#include "pinocchio-rust/math/eigen.hpp"


namespace pinocchio {


void rnea(const std::unique_ptr<Model>& model, std::unique_ptr<Data>& data,
          rust::Slice<const double> q, rust::Slice<const double> v, 
          rust::Slice<const double> a) {
  rnea(*model.get(), *data.get(), 
       Eigen::ConstVectorXdMap(q, model->nq), 
       Eigen::ConstVectorXdMap(v, model->nv), 
       Eigen::ConstVectorXdMap(a, model->nv));
}

}