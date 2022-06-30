#include "pinocchio-rust/algorithm/crba.hpp"
#include "pinocchio/algorithm/crba.hpp"

#include "pinocchio-rust/math/eigen.hpp"


namespace pinocchio {

void crba(const std::unique_ptr<Model>& model, std::unique_ptr<Data>& data,
          rust::Slice<const double> q) {
  crba(*model.get(), *data.get(), 
       Eigen::ConstVectorXdMap(q, model->nq));
  data->M.triangularView<Eigen::StrictlyLower>() 
      = data->M.transpose().triangularView<Eigen::StrictlyLower>();
}

void crbaMinimal(const std::unique_ptr<Model>& model, std::unique_ptr<Data>& data,
                 rust::Slice<const double> q) {
  crbaMinimal(*model.get(), *data.get(), 
              Eigen::ConstVectorXdMap(q, model->nq));
  data->M.triangularView<Eigen::StrictlyLower>() 
      = data->M.transpose().triangularView<Eigen::StrictlyLower>();
}


}