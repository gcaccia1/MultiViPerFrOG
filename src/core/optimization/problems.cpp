

#include "multiviperfrog/core/optimization/problems.h"

namespace core {
namespace optimization {

//----------------------------------------------------------------------------------------------------------------------
// CERES cost block functions
// error term 0 (m0) pseudo-measure of x5 for supervision
void BuildOptimizationProblemC2CMeasured0(const Eigen::Vector3d& Ca0__t__Ca0_Cb0_measured,        // m0, measure of x5
                                          const Eigen::Quaternion<double>& Ca0__q__Cb0_measured,  // same as above
                                          Eigen::Vector3d* Ca0__t__Ca0_Cb0_estim,                 // estimate of x5 as optimized
                                          Eigen::Quaternion<double>* Ca0__q__Cb0_estim,           // same as above
                                          struct config::ExperimentConfig* config,                // configuration file
                                          ceres::Problem* problem) {
  // build the problem
  ceres::LossFunction* loss_function = new ceres::ScaledLoss(nullptr, config->alpha_C2C0, ceres::TAKE_OWNERSHIP);
  ceres::Manifold* quaternion_manifold = new ceres::EigenQuaternionManifold;
  ceres::CostFunction* cost_function =
      core::optimization::cost_functions::TfMeasuredErrorterm::Create(Ca0__t__Ca0_Cb0_measured, Ca0__q__Cb0_measured);
  // add Tf cost to the residual block
  problem->AddResidualBlock(cost_function, loss_function, Ca0__t__Ca0_Cb0_estim->data(), Ca0__q__Cb0_estim->coeffs().data());
  // set the parameterization of the quaternion
  problem->SetManifold(Ca0__q__Cb0_estim->coeffs().data(), quaternion_manifold);
  // set any desired parameter block to be constant
  // NOTE: The parameter should not be set constant in other blocks, otherwise it will not be optimized!
}

// error term xx (m1) pseudo-measure of x6 for supervision
void BuildOptimizationProblemC2CMeasured1(const Eigen::Vector3d& Ca1__t__Ca1_Cb1_measured,        // m1, measure of x6
                                          const Eigen::Quaternion<double>& Ca1__q__Cb1_measured,  // same as above
                                          Eigen::Vector3d* Ca1__t__Ca1_Cb1_estim,                 // estimate of x6 as optimized
                                          Eigen::Quaternion<double>* Ca1__q__Cb1_estim,           // same as above
                                          struct config::ExperimentConfig* config,                // configuration file
                                          ceres::Problem* problem) {
  // build the problem
  ceres::LossFunction* loss_function = new ceres::ScaledLoss(nullptr, config->alpha_C2C1, ceres::TAKE_OWNERSHIP);
  ceres::Manifold* quaternion_manifold = new ceres::EigenQuaternionManifold;
  ceres::CostFunction* cost_function =
      core::optimization::cost_functions::TfMeasuredErrorterm::Create(Ca1__t__Ca1_Cb1_measured, Ca1__q__Cb1_measured);
  // add Tf cost to the residual block
  problem->AddResidualBlock(cost_function, loss_function, Ca1__t__Ca1_Cb1_estim->data(), Ca1__q__Cb1_estim->coeffs().data());
  // set the parameterization of the quaternion
  problem->SetManifold(Ca1__q__Cb1_estim->coeffs().data(), quaternion_manifold);
  // set any desired parameter block to be constant
  // NOTE: The parameter should not be set constant in other blocks, otherwise it will not be optimized!
}

// error term 1 bis (x6, x1, x2, x5). all parameters, just imposing the chain
void BuildOptimizationProblemTfChain(Eigen::Vector3d* Ca0__t__Ca0_Ca1,         // translation of x1 (parameter)
                                     Eigen::Quaternion<double>* Ca0__q__Ca1,   // quaternion of x1 (parameter)
                                     Eigen::Vector3d* Ca0__t__Ca0_Cb0,         // translation of X5 (constant)
                                     Eigen::Quaternion<double>* Ca0__q__Cb0,   // quaternion of X5 (constant)
                                     Eigen::Vector3d* Cb0__t__Cb0_Cb1,         // translation of x2 (parameter)
                                     Eigen::Quaternion<double>* Cb0__q__Cb1,   // quaternion of x2 (parameter)
                                     Eigen::Vector3d* Ca1__t__Ca1_Cb1,         // translation of x6 (parameter)
                                     Eigen::Quaternion<double>* Ca1__q__Cb1,   // quaternion of x6 (parameter)
                                     struct config::ExperimentConfig* config,  // configuration file
                                     ceres::Problem* problem) {
  // build the problem
  // setting for whole block
  ceres::LossFunction* loss_function = new ceres::ScaledLoss(nullptr, config->alpha_chain, ceres::TAKE_OWNERSHIP);
  ceres::Manifold* quaternion_manifold = new ceres::EigenQuaternionManifold;
  ceres::CostFunction* cost_function = core::optimization::cost_functions::TfChainErrorterm::Create();
  // add Tf cost to the residual block
  problem->AddResidualBlock(cost_function, loss_function, Ca0__t__Ca0_Ca1->data(), Ca0__q__Ca1->coeffs().data(), Ca0__t__Ca0_Cb0->data(),
                            Ca0__q__Cb0->coeffs().data(), Cb0__t__Cb0_Cb1->data(), Cb0__q__Cb1->coeffs().data(), Ca1__t__Ca1_Cb1->data(),
                            Ca1__q__Cb1->coeffs().data());

  // set the parameterization of the quaternion
  problem->SetManifold(Ca0__q__Ca1->coeffs().data(), quaternion_manifold);
  problem->SetManifold(Ca0__q__Cb0->coeffs().data(), quaternion_manifold);
  problem->SetManifold(Cb0__q__Cb1->coeffs().data(), quaternion_manifold);
  problem->SetManifold(Ca1__q__Cb1->coeffs().data(), quaternion_manifold);
  // set any desired parameter block to be constant
  //    problem->SetParameterBlockConstant(Ca0__t__Ca0_Ca1->data()); //(odometry Ca translation)
  //    problem->SetParameterBlockConstant(Ca0__q__Ca1->coeffs().data()); //(odometry Ca quaternion)
  //    problem->SetParameterBlockConstant(Ca0__t__Ca0_Cb0->data()); // (cam2cam time 0 translation)
  //    problem->SetParameterBlockConstant(Ca0__q__Cb0->coeffs().data()); // (cam2cam time 0 quaternion)
  //    problem->SetParameterBlockConstant(Cb0__t__Cb0_Cb1->data()); //(odometry Cb translation)
  //    problem->SetParameterBlockConstant(Cb0__q__Cb1->coeffs().data()); //(odometry Cb quaternion)
  //    problem->SetParameterBlockConstant(Ca1__t__Ca1_Cb1->data()); //(odometry Cb translation)
  //    problem->SetParameterBlockConstant(Ca1__q__Cb1->coeffs().data()); //(odometry Cb quaternion)
}

void BuildOptimizationProblemRigidTf1(const Eigen::Vector3d& Ca1__t__Ca1_Cb1,        // m1 (measure)
                                      const Eigen::Quaternion<double>& Ca1__q__Cb1,  // m1 (measure)
                                      Eigen::Vector3d* Ca0__t__Ca0_Ca1,              // translation of x1 (parameter)
                                      Eigen::Quaternion<double>* Ca0__q__Ca1,        // quaternion of x1 (parameter)
                                      Eigen::Vector3d* Ca0__t__Ca0_Cb0,              // translation of X5 (constant)
                                      Eigen::Quaternion<double>* Ca0__q__Cb0,        // quaternion of X5 (constant)
                                      Eigen::Vector3d* Cb0__t__Cb0_Cb1,              // translation of x2 (parameter)
                                      Eigen::Quaternion<double>* Cb0__q__Cb1,        // quaternion of x2 (parameter)
                                      struct config::ExperimentConfig* config,       // configuration file
                                      ceres::Problem* problem) {
  // build the problem
  // setting for whole block
  ceres::LossFunction* loss_function = new ceres::ScaledLoss(nullptr, config->alpha_chain, ceres::TAKE_OWNERSHIP);
  ceres::Manifold* quaternion_manifold = new ceres::EigenQuaternionManifold;
  ceres::CostFunction* cost_function = core::optimization::cost_functions::OdoFromRigidTf1Errorterm::Create(Ca1__t__Ca1_Cb1, Ca1__q__Cb1);
  // add Tf cost to the residual block
  problem->AddResidualBlock(cost_function, loss_function, Ca0__t__Ca0_Ca1->data(), Ca0__q__Ca1->coeffs().data(), Ca0__t__Ca0_Cb0->data(),
                            Ca0__q__Cb0->coeffs().data(), Cb0__t__Cb0_Cb1->data(), Cb0__q__Cb1->coeffs().data());
  // set the parameterization of the quaternion
  problem->SetManifold(Ca0__q__Ca1->coeffs().data(), quaternion_manifold);
  problem->SetManifold(Ca0__q__Cb0->coeffs().data(), quaternion_manifold);
  problem->SetManifold(Cb0__q__Cb1->coeffs().data(), quaternion_manifold);
  // set any desired parameter block to be constant
  //    problem->SetParameterBlockConstant(Ca0__t__Ca0_Ca1->data()); //(odometry Ca translation)
  //    problem->SetParameterBlockConstant(Ca0__q__Ca1->coeffs().data()); //(odometry Ca quaternion)
  //    problem->SetParameterBlockConstant(Ca0__t__Ca0_Cb0->data()); // (cam2cam time 0 translation)
  //    problem->SetParameterBlockConstant(Ca0__q__Cb0->coeffs().data()); // (cam2cam time 0 quaternion)
  //    problem->SetParameterBlockConstant(Cb0__t__Cb0_Cb1->data()); //(odometry Cb translation)
  //    problem->SetParameterBlockConstant(Cb0__q__Cb1->coeffs().data()); //(odometry Cb quaternion)
}

void BuildOptimizationProblemPointsA(const Eigen::Matrix4Xd& C0__t__C0_P0,                 // current points CamB, measure m5
                                     const Eigen::Matrix4Xd& C1__t__C1_P1,                 // biased flowed points (measures)
                                     Eigen::Vector3d* C0__t__C0_C1_estim,                  // transl. of x1 (parameter)
                                     Eigen::Quaternion<double>* C0__q__C1_estim,           // quat. of x1 (par.)
                                     Eigen::Matrix4Xd* C0__t__P0_P1,                       // true scene flow (parameter) Cb0_SF
                                     Eigen::Vector3d* C0self__t__C0self_C0other_estim,     // transl. of X5 (par.)
                                     Eigen::Quaternion<double>* C0self__q__C0other_estim,  // quat. of X5 (par.)
                                     struct config::ExperimentConfig* config,              // configuration file
                                     ceres::Problem* problem) {
  // build the problem
  //    ceres::LossFunction *loss_function = new ceres::ScaledLoss(nullptr, config->alpha_pointsA / config->num_points,
  ceres::LossFunction* loss_function = new ceres::ScaledLoss(nullptr, config->alpha_pointsA, ceres::TAKE_OWNERSHIP);
  ceres::Manifold* quaternion_manifold = new ceres::EigenQuaternionManifold;
  // settings for each point
  for (int i = 0; i < config->num_points; i++) {
    const Eigen::Vector3d C0__t__C0_P0i = C0__t__C0_P0.block(0, i, 3, 1);  // measure
    const Eigen::Vector3d C1__t__C1_P1i = C1__t__C1_P1.block(0, i, 3, 1);  // measure
    auto C0__t__P0_P1i = C0__t__P0_P1->block(0, i, 3, 1);                  // parameter
    // define cost (residual) for the point through custom cost function
    ceres::CostFunction* cost_function =
        core::optimization::cost_functions::OdoAndTrueflowFromBiasedflowErrortermA::Create(C0__t__C0_P0i, C1__t__C1_P1i);
    // add the point cost to the residual block
    problem->AddResidualBlock(cost_function, loss_function, C0__t__C0_C1_estim->data(), C0__q__C1_estim->coeffs().data(),
                              C0__t__P0_P1i.data(), C0self__t__C0self_C0other_estim->data(), C0self__q__C0other_estim->coeffs().data());
    // max and min bounds for C0__t__P0_P1i
    problem->SetParameterLowerBound(C0__t__P0_P1i.data(), 0, -config->max_abs_SF);
    problem->SetParameterLowerBound(C0__t__P0_P1i.data(), 1, -config->max_abs_SF);
    problem->SetParameterLowerBound(C0__t__P0_P1i.data(), 2, -config->max_abs_SF);
    problem->SetParameterUpperBound(C0__t__P0_P1i.data(), 0, config->max_abs_SF);
    problem->SetParameterUpperBound(C0__t__P0_P1i.data(), 1, config->max_abs_SF);
    problem->SetParameterUpperBound(C0__t__P0_P1i.data(), 2, config->max_abs_SF);
    // set any desired parameter block to be constant
    //        problem->SetParameterBlockConstant(C0__t__P0_P1i.data()); // (true scene flow points)
  }
  // set the parameterization of the quaternion
  problem->SetManifold(C0__q__C1_estim->coeffs().data(), quaternion_manifold);
  problem->SetManifold(C0self__q__C0other_estim->coeffs().data(), quaternion_manifold);
  // set any desired parameter block to be constant
  //    problem->SetParameterBlockConstant(C0__t__C0_C1_estim->data()); // (odometry translation)
  //    problem->SetParameterBlockConstant(C0__q__C1_estim->coeffs().data()); // (odometry quaternion)
  //    problem->SetParameterBlockConstant(C0self__t__C0self_C0other_estim->data()); // (cam2cam time 0 translation)
  //    problem->SetParameterBlockConstant(C0self__q__C0other_estim->coeffs().data()); // (cam2cam time 0 quaternion)
}

void BuildOptimizationProblemPointsB(const Eigen::Matrix4Xd& C0__t__C0_P0,        // current points CamB, measure m5
                                     const Eigen::Matrix4Xd& C1__t__C1_P1,        // biased flowed points measure m3
                                     Eigen::Vector3d* C0__t__C0_C1_estim,         // translation of x2 (parameter)
                                     Eigen::Quaternion<double>* C0__q__C1_estim,  // quaternion of x2 (parameter)
                                     Eigen::Matrix4Xd* C0__t__P0_P1,              // true scene flow (parameter) Cb0_SF
                                     struct config::ExperimentConfig* config,     // configuration file
                                     ceres::Problem* problem) {
  // build the problem
  // setting for whole block
//  ceres::LossFunction* loss_function = new ceres::ScaledLoss(nullptr, config->alpha_pointsB / config->num_points, ceres::TAKE_OWNERSHIP);
  ceres::LossFunction* loss_function = new ceres::ScaledLoss(nullptr, config->alpha_pointsB, ceres::TAKE_OWNERSHIP);
  ceres::Manifold* quaternion_manifold = new ceres::EigenQuaternionManifold;
  // settings for each point
  for (int i = 0; i < config->num_points; i++) {
    const Eigen::Vector3d C0__t__C0_P0i = C0__t__C0_P0.block(0, i, 3, 1);  // measure
    const Eigen::Vector3d C1__t__C1_P1i = C1__t__C1_P1.block(0, i, 3, 1);  // measure
    auto C0__t__P0_P1i = C0__t__P0_P1->block(0, i, 3, 1);                  // parameter
    // define cost (residual) for the point through custom cost function
    ceres::CostFunction* cost_function =
        core::optimization::cost_functions::OdoAndTrueflowFromBiasedflowErrortermB::Create(C0__t__C0_P0i, C1__t__C1_P1i);
    // add the point cost to the residual block
    problem->AddResidualBlock(cost_function, loss_function, C0__t__C0_C1_estim->data(), C0__q__C1_estim->coeffs().data(),
                              C0__t__P0_P1i.data());
  // max and min bounds for C0__t__P0_P1i
    problem->SetParameterLowerBound(C0__t__P0_P1i.data(), 0, -config->max_abs_SF);
    problem->SetParameterLowerBound(C0__t__P0_P1i.data(), 1, -config->max_abs_SF);
    problem->SetParameterLowerBound(C0__t__P0_P1i.data(), 2, -config->max_abs_SF);
    problem->SetParameterUpperBound(C0__t__P0_P1i.data(), 0, config->max_abs_SF);
    problem->SetParameterUpperBound(C0__t__P0_P1i.data(), 1, config->max_abs_SF);
    problem->SetParameterUpperBound(C0__t__P0_P1i.data(), 2, config->max_abs_SF);
    // set any desired parameter block to be constant
    //        problem->SetParameterBlockConstant(C0__t__P0_P1i.data()); // (true scene flow)
  }
  // set the parameterization of the quaternion
  problem->SetManifold(C0__q__C1_estim->coeffs().data(), quaternion_manifold);
  // set any desired parameter block to be constant
  //    problem->SetParameterBlockConstant(C0__t__C0_C1_estim->data()); // (odometry translation)
  //    problem->SetParameterBlockConstant(C0__q__C1_estim->coeffs().data()); // (odometry quaternion)
}

// NOTE: does it make sense to call it self=source and other=target??
void BuildOptimizationProblemDataAssC0(const Eigen::Matrix4Xd& Cself__t__Cself_P,          // current points CamA, measure m4
                                       const Eigen::Matrix4Xd& Cother__t__Cother_P,        // current points CamB, measure m5
                                       Eigen::Vector3d* Cself__t__Cself_Cother_estim,      // transl. of X5 (par.)
                                       Eigen::Quaternion<double>* Cself__q__Cother_estim,  // quat. of X5 (par.)
                                       struct config::ExperimentConfig* config,            // configuration file
                                       ceres::Problem* problem) {
  // build the problem
  //    ceres::LossFunction *loss_function = new ceres::ScaledLoss(nullptr, config->alpha_DataAssC0 / config->num_points,
  ceres::LossFunction* loss_function = new ceres::ScaledLoss(nullptr, config->alpha_DataAssC0, ceres::TAKE_OWNERSHIP);
  ceres::Manifold* quaternion_manifold = new ceres::EigenQuaternionManifold;
  // settings for each point
  for (int i = 0; i < config->num_points; i++) {
    const Eigen::Vector3d Cself__t__Cself_Pi = Cself__t__Cself_P.block(0, i, 3, 1);      // measure m4
    const Eigen::Vector3d Cother__t__Cother_Pi = Cother__t__Cother_P.block(0, i, 3, 1);  // measure m5
    // define cost (residual) for the point through custom cost function
    ceres::CostFunction* cost_function =
        core::optimization::cost_functions::DataAssociationErrorterm::Create(Cself__t__Cself_Pi, Cother__t__Cother_Pi);
    // add the point cost to the residual block
    problem->AddResidualBlock(cost_function, loss_function, Cself__t__Cself_Cother_estim->data(), Cself__q__Cother_estim->coeffs().data());
  }
  // set the parameterization of the quaternion
  problem->SetManifold(Cself__q__Cother_estim->coeffs().data(), quaternion_manifold);
  // set any desired parameter block to be constant
}

void BuildOptimizationProblemDataAssC1(const Eigen::Matrix4Xd& Cself__t__Cself_P,          // current points CamA, measure m4
                                       const Eigen::Matrix4Xd& Cother__t__Cother_P,        // current points CamB, measure m5
                                       Eigen::Vector3d* Cself__t__Cself_Cother_estim,      // transl. of X5 (par.)
                                       Eigen::Quaternion<double>* Cself__q__Cother_estim,  // quat. of X5 (par.)
                                       struct config::ExperimentConfig* config,            // configuration file
                                       ceres::Problem* problem) {
  // build the problem
  //    ceres::LossFunction *loss_function = new ceres::ScaledLoss(nullptr, config->alpha_DataAssC1 / config->num_points,
  ceres::LossFunction* loss_function = new ceres::ScaledLoss(nullptr, config->alpha_DataAssC1, ceres::TAKE_OWNERSHIP);
  ceres::Manifold* quaternion_manifold = new ceres::EigenQuaternionManifold;
  // settings for each point
  for (int i = 0; i < config->num_points; i++) {
    const Eigen::Vector3d Cself__t__Cself_Pi = Cself__t__Cself_P.block(0, i, 3, 1);      // measure m4
    const Eigen::Vector3d Cother__t__Cother_Pi = Cother__t__Cother_P.block(0, i, 3, 1);  // measure m5
    // define cost (residual) for the point through custom cost function
    ceres::CostFunction* cost_function =
        core::optimization::cost_functions::DataAssociationErrorterm::Create(Cself__t__Cself_Pi, Cother__t__Cother_Pi);
    // add the point cost to the residual block
    problem->AddResidualBlock(cost_function, loss_function, Cself__t__Cself_Cother_estim->data(), Cself__q__Cother_estim->coeffs().data());
  }
  // set the parameterization of the quaternion
  problem->SetManifold(Cself__q__Cother_estim->coeffs().data(), quaternion_manifold);
  // set any desired parameter block to be constant
}

void BuildOptimizationProblemKnownSF(const Eigen::Matrix4Xd& C0__t__P0_P1_knowledge,      // Cb0_SF knowledge
                                     const std::vector<bool>& C0__t__P0_P1_columns_mask,  // mask for the known values
                                     Eigen::Matrix4Xd* C0__t__P0_P1,                      // true scene flow (parameter) Cb0_SF
                                     struct config::ExperimentConfig* config,             // configuration file
                                     ceres::Problem* problem) {
  // build the problem
  // setting for whole block
  //        // initialize with total number of true values in mask
  //        const int config.num_points_mask = std::count(C0__t__P0_P1_columns_mask.begin(), C0__t__P0_P1_columns_mask.end(), true);
  //    ceres::LossFunction *loss_function = new ceres::ScaledLoss(nullptr, config->alpha_SF / config->num_points,
  ceres::LossFunction* loss_function = new ceres::ScaledLoss(nullptr, config->alpha_SF, ceres::TAKE_OWNERSHIP);
  // settings for each point
  for (int i = 0; i < config->num_points; i++) {
    if (!C0__t__P0_P1_columns_mask[i]) {
      continue;
    }
    const Eigen::Vector3d C0__t__P0_P1i_knowledge = C0__t__P0_P1_knowledge.block(0, i, 3, 1);  // measure
    auto C0__t__P0_P1i = C0__t__P0_P1->block(0, i, 3, 1);                                      // parameter
    // define cost (residual) for the point through custom cost function
    ceres::CostFunction* cost_function = core::optimization::cost_functions::SFKnowledgeErrorterm::Create(C0__t__P0_P1i_knowledge);
    // add the point cost to the residual block
    problem->AddResidualBlock(cost_function, loss_function, C0__t__P0_P1i.data());
    // set any desired parameter block to be constant
  }
}

void BuildOptimizationProblemOdomMeasuredA(const Eigen::Vector3d& C0__t__C0_C1_measured,         // m6, measure of x1
                                           const Eigen::Quaternion<double>& C0__q__C1_measured,  // same as above
                                           Eigen::Vector3d* C0__t__C0_C1_estim,                  // estimate of x1 as optimized
                                           Eigen::Quaternion<double>* C0__q__C1_estim,           // same as above
                                           struct config::ExperimentConfig* config,              // configuration file
                                           ceres::Problem* problem) {
  // build the problem
  ceres::LossFunction* loss_function = new ceres::ScaledLoss(nullptr, config->alpha_odoA, ceres::TAKE_OWNERSHIP);
  ceres::Manifold* quaternion_manifold = new ceres::EigenQuaternionManifold;
  ceres::CostFunction* cost_function =
      core::optimization::cost_functions::TfMeasuredErrorterm::Create(C0__t__C0_C1_measured, C0__q__C1_measured);
  // add Tf cost to the residual block
  problem->AddResidualBlock(cost_function, loss_function, C0__t__C0_C1_estim->data(), C0__q__C1_estim->coeffs().data());
  // set the parameterization of the quaternion
  problem->SetManifold(C0__q__C1_estim->coeffs().data(), quaternion_manifold);
  // set any desired parameter block to be constant
  // NOTE: The parameter should not be set constant in other blocks, otherwise it will not be optimized!
}

void BuildOptimizationProblemOdomMeasuredB(const Eigen::Vector3d& C0__t__C0_C1_measured,         // m7, measure of x2
                                           const Eigen::Quaternion<double>& C0__q__C1_measured,  // same as above
                                           Eigen::Vector3d* C0__t__C0_C1_estim,                  // estimate of X2 as optimized
                                           Eigen::Quaternion<double>* C0__q__C1_estim,           // same as above
                                           struct config::ExperimentConfig* config,              // configuration file
                                           ceres::Problem* problem) {
  // build the problem
  ceres::LossFunction* loss_function = new ceres::ScaledLoss(nullptr, config->alpha_odoB, ceres::TAKE_OWNERSHIP);
  ceres::Manifold* quaternion_manifold = new ceres::EigenQuaternionManifold;
  ceres::CostFunction* cost_function =
      core::optimization::cost_functions::TfMeasuredErrorterm::Create(C0__t__C0_C1_measured, C0__q__C1_measured);
  // add Tf cost to the residual block
  problem->AddResidualBlock(cost_function, loss_function, C0__t__C0_C1_estim->data(), C0__q__C1_estim->coeffs().data());
  // set the parameterization of the quaternion
  problem->SetManifold(C0__q__C1_estim->coeffs().data(), quaternion_manifold);
  // set any desired parameter block to be constant
  // NOTE: The parameter should not be set constant in other blocks, otherwise it will not be optimized!
}

void BuildOptimizationProblemTfandStaticLabels(const Eigen::Matrix4Xd& C0__t__C0_P0,  // current points CamB, measure m5
                                               const Eigen::Matrix4Xd& C1__t__C1_P1,  // biased flowed points measure m3
                                               const double& lambda_deformation, const double& lambda_regularization,
                                               Eigen::Vector3d* C0__t__C0_C1_estim,         // translation of x2 (parameter)
                                               Eigen::Quaternion<double>* C0__q__C1_estim,  // quaternion of x2 (parameter)
                                               Eigen::VectorXd* rigidity_labels,  // rigidity labels (1= fully rigid, 0= fully deformable)
                                               struct config::ExperimentConfig* config,  // configuration file
                                               ceres::Problem* problem) {
  // build the problem
  // setting for whole block

  //          TODO check: does this go in the for loop?
  ceres::LossFunction* loss_function = nullptr;
  if (config->use_huber_loss) {
    loss_function =
        new ceres::HuberLoss(config->huber_loss_threshold);  // TODO: test other loss functions (e.g. CauchyLoss, SoftLOneLoss, etc.
  }
  ceres::Manifold* quaternion_manifold = new ceres::EigenQuaternionManifold;
  // settings for each point
  for (int i = 0; i < config->num_points; i++) {                           // TODO: structure labels discretzation with superpixels
    const Eigen::Vector3d C0__t__C0_P0i = C0__t__C0_P0.block(0, i, 3, 1);  // measure
    const Eigen::Vector3d C1__t__C1_P1i = C1__t__C1_P1.block(0, i, 3, 1);  // measure
    auto rigidity_label = rigidity_labels->block(i, 0, 1, 1);              // parameter

    // weight the influence of each point with the rigidity label
    // TODO check usage of [0], also inside of the cost function
    if (!config->use_huber_loss) {
      double loss_per_point = rigidity_label.data()[0] / config->num_points;
      loss_function = new ceres::ScaledLoss(nullptr, loss_per_point, ceres::TAKE_OWNERSHIP);
    }

    // define cost (residual) for the point through custom cost function
    ceres::CostFunction* cost_function = core::optimization::cost_functions::TfAndStaticLabelsFromCorrespondencesErrorTerm::Create(
        C0__t__C0_P0i, C1__t__C1_P1i, lambda_deformation, lambda_regularization);
    // add the point cost to the residual block
    problem->AddResidualBlock(cost_function, loss_function, C0__t__C0_C1_estim->data(), C0__q__C1_estim->coeffs().data(),
                              rigidity_label.data());

    // set bounds for the rigidity label
    problem->SetParameterLowerBound(rigidity_label.data(), 0, 0.000000001);
    problem->SetParameterUpperBound(rigidity_label.data(), 0, 1.0);

    // set any desired parameter block to be constant
    if (config->use_huber_loss) {
      problem->SetParameterBlockConstant(rigidity_label.data());
    } else if (i % config->rigidity_label_update_interval != 0) {  // set the rigidity label constant at interval
      problem->SetParameterBlockConstant(rigidity_label.data());
    }
  }
  // set the parameterization of the quaternion
  problem->SetManifold(C0__q__C1_estim->coeffs().data(), quaternion_manifold);
  // set any desired parameter block to be constant
  //    problem->SetParameterBlockConstant(C0__t__C0_C1_estim->data()); // (odometry translation)
  //    problem->SetParameterBlockConstant(C0__q__C1_estim->coeffs().data()); // (odometry quaternion)
}

void BuildOptimizationProblemTfandStaticLabelsExplicitSF(const Eigen::Matrix4Xd& C0__t__C0_P0,  // current points CamB, measure m5
                                                         const Eigen::Matrix4Xd& C1__t__C1_P1,  // biased flowed points measure m3
                                                         const double& lambda_deformation, const double& lambda_regularization,
                                                         Eigen::Vector3d* C0__t__C0_C1_estim,         // translation of x2 (parameter)
                                                         Eigen::Quaternion<double>* C0__q__C1_estim,  // quaternion of x2 (parameter)
                                                         Eigen::Matrix4Xd* C0__t__P0_P1,              // true scene flow (parameter) Cb0_SF
                                                         Eigen::VectorXd* rigidity_labels,  // rigidity labels (1= fully rigid, 0= fully deformable)
                                                         struct config::ExperimentConfig* config,  // configuration file
                                                         ceres::Problem* problem) {
  // build the problem
  // setting for whole block

  //          TODO check: does this go in the for loop?
  ceres::LossFunction* loss_function = nullptr;
  if (config->use_huber_loss) {
    loss_function =
        new ceres::HuberLoss(config->huber_loss_threshold);  // TODO: test other loss functions (e.g. CauchyLoss, SoftLOneLoss, etc.
  }
  ceres::Manifold* quaternion_manifold = new ceres::EigenQuaternionManifold;
  // settings for each point
  for (int i = 0; i < config->num_points; i++) {                           // TODO: structure labels discretzation with superpixels
    const Eigen::Vector3d C0__t__C0_P0i = C0__t__C0_P0.block(0, i, 3, 1);  // measure
    const Eigen::Vector3d C1__t__C1_P1i = C1__t__C1_P1.block(0, i, 3, 1);  // measure
    auto rigidity_label = rigidity_labels->block(i, 0, 1, 1);              // parameter
    auto C0__t__P0_P1i = C0__t__P0_P1->block(0, i, 3, 1);                  // parameter

    // weight the influence of each point with the rigidity label
    // TODO check usage of [0], also inside of the cost function
    if (!config->use_huber_loss) {
      double loss_per_point = rigidity_label.data()[0] / config->num_points;
      loss_function = new ceres::ScaledLoss(nullptr, loss_per_point, ceres::TAKE_OWNERSHIP);
    }

    // define cost (residual) for the point through custom cost function
    ceres::CostFunction* cost_function = core::optimization::cost_functions::TfAndStaticLabelsFromCorrespondencesErrorTermExplicitSF::Create(
        C0__t__C0_P0i, C1__t__C1_P1i, lambda_deformation, lambda_regularization);
    // add the point cost to the residual block
    problem->AddResidualBlock(cost_function, loss_function, C0__t__C0_C1_estim->data(), C0__q__C1_estim->coeffs().data(),
                              C0__t__P0_P1i.data(), rigidity_label.data());

    // set bounds for the rigidity label
    problem->SetParameterLowerBound(rigidity_label.data(), 0, 0.000000001);
    problem->SetParameterUpperBound(rigidity_label.data(), 0, 1.0);
    // set any desired parameter block to be constant
    if (config->use_huber_loss) {
      problem->SetParameterBlockConstant(rigidity_label.data());
    } else if (i % config->rigidity_label_update_interval != 0) {  // set the rigidity label constant at interval
      problem->SetParameterBlockConstant(rigidity_label.data());
    }
  }
  // set the parameterization of the quaternion
  problem->SetManifold(C0__q__C1_estim->coeffs().data(), quaternion_manifold);
  // set any desired parameter block to be constant
  //    problem->SetParameterBlockConstant(C0__t__C0_C1_estim->data()); // (odometry translation)
  //    problem->SetParameterBlockConstant(C0__q__C1_estim->coeffs().data()); // (odometry quaternion)
}


void BuildOptimizationProblemPointsPlusLabels(const Eigen::Matrix4Xd& C0__t__C0_P0,  // current points CamB, measure m5
                                               const Eigen::Matrix4Xd& C1__t__C1_P1,  // biased flowed points measure m3
                                               Eigen::Vector3d* C0__t__C0_C1_estim,         // translation of x2 (parameter)
                                               Eigen::Quaternion<double>* C0__q__C1_estim,  // quaternion of x2 (parameter)
                                               Eigen::Matrix4Xd* C0__t__P0_P1,              // true scene flow (parameter) Cb0_SF
                                               Eigen::VectorXd* rigidity_labels,  // rigidity labels (1= fully rigid, 0= fully deformable)
                                               double* labels_to_SF_scaling,  // scaling factor for the labels
                                               struct config::ExperimentConfig* config,  // configuration file
                                               ceres::Problem* problem) {
  // build the problem
  // setting for whole block
  ceres::LossFunction* loss_function = new ceres::ScaledLoss(nullptr, config->alpha_pointsB, ceres::TAKE_OWNERSHIP);
  ceres::Manifold* quaternion_manifold = new ceres::EigenQuaternionManifold;
  // settings for each point
  for (int i = 0; i < config->num_points; i++) {                           // TODO: structure labels discretzation with superpixels
    const Eigen::Vector3d C0__t__C0_P0i = C0__t__C0_P0.block(0, i, 3, 1);  // measure
    const Eigen::Vector3d C1__t__C1_P1i = C1__t__C1_P1.block(0, i, 3, 1);  // measure
    auto rigidity_label = rigidity_labels->block(i, 0, 1, 1);              // parameter
    auto C0__t__P0_P1i = C0__t__P0_P1->block(0, i, 3, 1);                  // parameter

    // define cost (residual) for the point through custom cost function
    ceres::CostFunction* cost_function = core::optimization::cost_functions::OdoAndTrueflowFromBiasedflowErrortermPlusLabels::Create(
        C0__t__C0_P0i, C1__t__C1_P1i);
    // add the point cost to the residual block
    problem->AddResidualBlock(cost_function, loss_function, C0__t__C0_C1_estim->data(), C0__q__C1_estim->coeffs().data(),
                              C0__t__P0_P1i.data(), rigidity_label.data(), labels_to_SF_scaling);

    // set bounds for the rigidity label
    problem->SetParameterLowerBound(rigidity_label.data(), 0, 0.000000001);
    problem->SetParameterUpperBound(rigidity_label.data(), 0, 1.0);
  }
  // set the parameterization of the quaternion
  problem->SetManifold(C0__q__C1_estim->coeffs().data(), quaternion_manifold);
  // set any desired parameter block to be constant
  //    problem->SetParameterBlockConstant(C0__t__C0_C1_estim->data()); // (odometry translation)
  //    problem->SetParameterBlockConstant(C0__q__C1_estim->coeffs().data()); // (odometry quaternion)
}

void BuildOptimizationProblemRCMOdo(Eigen::Vector3d* C0__t__C0_C1_estim,         // estimate of odometry as optimized
                                    Eigen::Quaternion<double>* C0__q__C1_estim,  // same as above
                                    struct config::ExperimentConfig* config,     // configuration file
                                    ceres::Problem* problem) {
  // build the problem
  ceres::LossFunction* loss_function = new ceres::ScaledLoss(nullptr, config->alpha_RCM, ceres::TAKE_OWNERSHIP);
  ceres::Manifold* quaternion_manifold = new ceres::EigenQuaternionManifold;
  ceres::CostFunction* cost_function = core::optimization::cost_functions::RCMConstraintErrorterm::Create();
  // add Tf cost to the residual block
  problem->AddResidualBlock(cost_function, loss_function, C0__t__C0_C1_estim->data(), C0__q__C1_estim->coeffs().data());
  // set the parameterization of the quaternion
  problem->SetManifold(C0__q__C1_estim->coeffs().data(), quaternion_manifold);
  // set any desired parameter block to be constant
  // NOTE: The parameter should not be set constant in other blocks, otherwise it will not be optimized!
}

}  // namespace optimization
}  // namespace core
