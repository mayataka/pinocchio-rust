#include "pinocchio-rust/algorithm/rnea.hpp"
#include "pinocchio/algorithm/rnea.hpp"

#include "pinocchio-rust/math/eigen.hpp"


namespace pinocchio {


void rnea(const std::unique_ptr<Model>& model, std::unique_ptr<Data>& data,
          const rust::Vec<double>& q,
          const rust::Vec<double>& v,
          const rust::Vec<double>& a) {
  rnea(*model.get(), *data.get(), 
       Eigen::StdVecToVectorXdMap(q, model->nq), 
       Eigen::StdVecToVectorXdMap(v, model->nv), 
       Eigen::StdVecToVectorXdMap(a, model->nv));
}

}