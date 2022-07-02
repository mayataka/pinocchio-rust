#include "pinocchio-rust/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/jacobian.hpp"

#include "pinocchio-rust/math/eigen.hpp"
#include "pinocchio-rust/algorithm/reference_frame.hpp"


namespace pinocchio {

void computeJointJacobians(const std::unique_ptr<Model>& model,
                           std::unique_ptr<Data>& data,
                           rust::Slice<const double> q) {
  computeJointJacobians(*model.get(), *data.get(), 
                        Eigen::ConstVectorXdMap(q, model->nq));
}

void getJointJacobian(const std::unique_ptr<Model>& model,
                      std::unique_ptr<Data>& data, 
                      const std::uint32_t joint_id, const std::uint32_t rf, 
                      rust::Slice<double> J) {
  getJointJacobian(*model.get(), *data.get(), joint_id,  
                   ReferenceFrameFromUint32(rf), 
                   Eigen::MatrixXdMap(J, 6, model->nv)); 
}

void computeJointJacobian(const std::unique_ptr<Model>& model,
                          std::unique_ptr<Data>& data,
                          rust::Slice<const double> q, 
                          const std::uint32_t joint_id,  
                          rust::Slice<double> J) {
  computeJointJacobian(*model.get(), *data.get(), 
                       Eigen::ConstVectorXdMap(q, model->nq), 
                       joint_id, Eigen::MatrixXdMap(J, 6, model->nv)); 
}

void computeJointJacobiansTimeVariation(const std::unique_ptr<Model>& model,
                                        std::unique_ptr<Data>& data,
                                        rust::Slice<const double> q,
                                        rust::Slice<const double> v) {
  computeJointJacobiansTimeVariation(*model.get(), *data.get(), 
                                     Eigen::ConstVectorXdMap(q, model->nq),
                                     Eigen::ConstVectorXdMap(v, model->nv));
}

void getJointJacobianTimeVariation(const std::unique_ptr<Model>& model,
                                   std::unique_ptr<Data>& data,
                                   const std::uint32_t joint_id, const std::uint32_t rf,
                                   rust::Slice<double> dJ) {
  getJointJacobianTimeVariation(*model.get(), *data.get(), joint_id, 
                                ReferenceFrameFromUint32(rf), 
                                Eigen::MatrixXdMap(dJ, 6, model->nv)); 
}

rust::Vec<std::uint32_t> jointJacobianSize(const std::unique_ptr<Model>& model) {
  return rust::Vec<std::uint32_t>({6, static_cast<std::uint32_t>(model->nv)});
}

rust::Vec<std::uint32_t> jointJacobianTimeVaryationSize(const std::unique_ptr<Model>& model) {
  return rust::Vec<std::uint32_t>({6, static_cast<std::uint32_t>(model->nv)});
}

}