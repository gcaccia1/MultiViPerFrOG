

#ifndef CORE_OPTIMIZATION_PROBLEMS_H
#define CORE_OPTIMIZATION_PROBLEMS_H

#include <ceres/ceres.h>
#include <iostream>
#include "multiviperfrog/config/ceres_config.h"
#include "multiviperfrog/config/experiment_config.h"
#include "multiviperfrog/config/optimization_data.h"
#include "multiviperfrog/core/optimization/cost_functions/RCM_constraint_errorterm.h"
#include "multiviperfrog/core/optimization/cost_functions/SF_knowledge_errorterm.h"
#include "multiviperfrog/core/optimization/cost_functions/Tf_and_staticlabels_from_correspondences_errorterm.h"
#include "multiviperfrog/core/optimization/cost_functions/Tf_and_staticlabels_from_correspondences_errorterm_explicit_SF.h"
#include "multiviperfrog/core/optimization/cost_functions/Tf_chain_errorterm.h"
#include "multiviperfrog/core/optimization/cost_functions/Tf_measured_errorterm.h"
#include "multiviperfrog/core/optimization/cost_functions/data_association_errorterm.h"
#include "multiviperfrog/core/optimization/cost_functions/odo_and_trueflow_from_biasedflow_errorterm_a.h"
#include "multiviperfrog/core/optimization/cost_functions/odo_and_trueflow_from_biasedflow_errorterm_b.h"
#include "multiviperfrog/core/optimization/cost_functions/odo_and_trueflow_from_biasedflow_errorterm_b_with_RCM.h"
#include "multiviperfrog/core/optimization/cost_functions/odo_from_rigidTf1_errorterm.h"
#include "multiviperfrog/core/optimization/cost_functions/odo_and_trueflow_from_biasedflow_plus_labels_errorterm.h"

namespace core {
namespace optimization {
void BuildOptimizationProblemC2CMeasured0(const Eigen::Vector3d& Ca0__t__Ca0_Cb0_measured,        // m0, measure of x5
                                          const Eigen::Quaternion<double>& Ca0__q__Cb0_measured,  // same as above
                                          Eigen::Vector3d* Ca0__t__Ca0_Cb0_estim,                 // estimate of x5 as optimized
                                          Eigen::Quaternion<double>* Ca0__q__Cb0_estim,           // same as above
                                          struct config::ExperimentConfig* config,                // configuration file
                                          ceres::Problem* problem);

void BuildOptimizationProblemC2CMeasured1(const Eigen::Vector3d& Ca1__t__Ca1_Cb1_measured,        // m1, measure of x6
                                          const Eigen::Quaternion<double>& Ca1__q__Cb1_measured,  // same as above
                                          Eigen::Vector3d* Ca1__t__Ca1_Cb1_estim,                 // estimate of x6 as optimized
                                          Eigen::Quaternion<double>* Ca1__q__Cb1_estim,           // same as above
                                          struct config::ExperimentConfig* config,                // configuration file
                                          ceres::Problem* problem);

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
                                     ceres::Problem* problem);

void BuildOptimizationProblemRigidTf1(const Eigen::Vector3d& Ca1__t__Ca1_Cb1,        // m1 (measure)
                                      const Eigen::Quaternion<double>& Ca1__q__Cb1,  // m1 (measure)
                                      Eigen::Vector3d* Ca0__t__Ca0_Ca1,              // translation of x1 (parameter)
                                      Eigen::Quaternion<double>* Ca0__q__Ca1,        // quaternion of x1 (parameter)
                                      Eigen::Vector3d* Ca0__t__Ca0_Cb0,              // translation of X5 (constant)
                                      Eigen::Quaternion<double>* Ca0__q__Cb0,        // quaternion of X5 (constant)
                                      Eigen::Vector3d* Cb0__t__Cb0_Cb1,              // translation of x2 (parameter)
                                      Eigen::Quaternion<double>* Cb0__q__Cb1,        // quaternion of x2 (parameter)
                                      struct config::ExperimentConfig* config,       // configuration file
                                      ceres::Problem* problem);

void BuildOptimizationProblemPointsA(const Eigen::Matrix4Xd& C0__t__C0_P0,                 // current points CamB, measure m5
                                     const Eigen::Matrix4Xd& C1__t__C1_P1,                 // biased flowed points (measures)
                                     Eigen::Vector3d* C0__t__C0_C1_estim,                  // transl. of x1 (parameter)
                                     Eigen::Quaternion<double>* C0__q__C1_estim,           // quat. of x1 (par.)
                                     Eigen::Matrix4Xd* C0__t__P0_P1,                       // true scene flow (parameter) Cb0_SF
                                     Eigen::Vector3d* C0self__t__C0self_C0other_estim,     // transl. of X5 (par.)
                                     Eigen::Quaternion<double>* C0self__q__C0other_estim,  // quat. of X5 (par.)
                                     struct config::ExperimentConfig* config,              // configuration file
                                     ceres::Problem* problem);

void BuildOptimizationProblemPointsB(const Eigen::Matrix4Xd& C0__t__C0_P0,        // current points CamB, measure m5
                                     const Eigen::Matrix4Xd& C1__t__C1_P1,        // biased flowed points measure m3
                                     Eigen::Vector3d* C0__t__C0_C1_estim,         // translation of x2 (parameter)
                                     Eigen::Quaternion<double>* C0__q__C1_estim,  // quaternion of x2 (parameter)
                                     Eigen::Matrix4Xd* C0__t__P0_P1,              // true scene flow (parameter) Cb0_SF
                                     struct config::ExperimentConfig* config,     // configuration file
                                     ceres::Problem* problem);

// NOTE: does it make sense to call it self=source and other=target??
void BuildOptimizationProblemDataAssC0(const Eigen::Matrix4Xd& Cself__t__Cself_P,          // current points CamA, measure m4
                                       const Eigen::Matrix4Xd& Cother__t__Cother_P,        // current points CamB, measure m5
                                       Eigen::Vector3d* Cself__t__Cself_Cother_estim,      // transl. of X5 (par.)
                                       Eigen::Quaternion<double>* Cself__q__Cother_estim,  // quat. of X5 (par.)
                                       struct config::ExperimentConfig* config,            // configuration file
                                       ceres::Problem* problem);

void BuildOptimizationProblemDataAssC1(const Eigen::Matrix4Xd& Cself__t__Cself_P,          // current points CamA, measure m4
                                       const Eigen::Matrix4Xd& Cother__t__Cother_P,        // current points CamB, measure m5
                                       Eigen::Vector3d* Cself__t__Cself_Cother_estim,      // transl. of X5 (par.)
                                       Eigen::Quaternion<double>* Cself__q__Cother_estim,  // quat. of X5 (par.)
                                       struct config::ExperimentConfig* config,            // configuration file
                                       ceres::Problem* problem);

void BuildOptimizationProblemKnownSF(const Eigen::Matrix4Xd& C0__t__P0_P1_knowledge,      // Cb0_SF knowledge
                                     const std::vector<bool>& C0__t__P0_P1_columns_mask,  // mask for the known values
                                     Eigen::Matrix4Xd* C0__t__P0_P1,                      // true scene flow (parameter) Cb0_SF
                                     struct config::ExperimentConfig* config,             // configuration file
                                     ceres::Problem* problem);

void BuildOptimizationProblemOdomMeasuredA(const Eigen::Vector3d& C0__t__C0_C1_measured,         // m6, measure of x1
                                           const Eigen::Quaternion<double>& C0__q__C1_measured,  // same as above
                                           Eigen::Vector3d* C0__t__C0_C1_estim,                  // estimate of x1 as optimized
                                           Eigen::Quaternion<double>* C0__q__C1_estim,           // same as above
                                           struct config::ExperimentConfig* config,              // configuration file
                                           ceres::Problem* problem);

void BuildOptimizationProblemOdomMeasuredB(const Eigen::Vector3d& C0__t__C0_C1_measured,         // m7, measure of x2
                                           const Eigen::Quaternion<double>& C0__q__C1_measured,  // same as above
                                           Eigen::Vector3d* C0__t__C0_C1_estim,                  // estimate of X2 as optimized
                                           Eigen::Quaternion<double>* C0__q__C1_estim,           // same as above
                                           struct config::ExperimentConfig* config,              // configuration file
                                           ceres::Problem* problem);

void BuildOptimizationProblemTfandStaticLabels(const Eigen::Matrix4Xd& C0__t__C0_P0,  // current points CamB, measure m5
                                               const Eigen::Matrix4Xd& C1__t__C1_P1,  // biased flowed points measure m3
                                               const double& lambda_deformation, const double& lambda_regularization,
                                               Eigen::Vector3d* C0__t__C0_C1_estim,         // translation of x2 (parameter)
                                               Eigen::Quaternion<double>* C0__q__C1_estim,  // quaternion of x2 (parameter)
                                               Eigen::VectorXd* rigidity_labels,  // rigidity labels (1= fully rigid, 0= fully deformable)
                                               struct config::ExperimentConfig* config,  // configuration file
                                               ceres::Problem* problem);

void BuildOptimizationProblemTfandStaticLabelsExplicitSF(const Eigen::Matrix4Xd& C0__t__C0_P0,  // current points CamB, measure m5
                                               const Eigen::Matrix4Xd& C1__t__C1_P1,  // biased flowed points measure m3
                                               const double& lambda_deformation, const double& lambda_regularization,
                                               Eigen::Vector3d* C0__t__C0_C1_estim,         // translation of x2 (parameter)
                                               Eigen::Quaternion<double>* C0__q__C1_estim,  // quaternion of x2 (parameter)
                                               Eigen::Matrix4Xd* C0__t__P0_P1,              // true scene flow (parameter) Cb0_SF
                                               Eigen::VectorXd* rigidity_labels,  // rigidity labels (1= fully rigid, 0= fully deformable)
                                               struct config::ExperimentConfig* config,  // configuration file
                                               ceres::Problem* problem);

void BuildOptimizationProblemPointsPlusLabels(const Eigen::Matrix4Xd& C0__t__C0_P0,  // current points CamB, measure m5
                                              const Eigen::Matrix4Xd& C1__t__C1_P1,  // biased flowed points measure m3
                                              Eigen::Vector3d* C0__t__C0_C1_estim,         // translation of x2 (parameter)
                                              Eigen::Quaternion<double>* C0__q__C1_estim,  // quaternion of x2 (parameter)
                                              Eigen::Matrix4Xd* C0__t__P0_P1,              // true scene flow (parameter) Cb0_SF
                                              Eigen::VectorXd* rigidity_labels,  // rigidity labels (1= fully rigid, 0= fully deformable)
                                              double* labels_to_SF_scaling,  // scaling factor for the labels
                                              struct config::ExperimentConfig* config,  // configuration file
                                              ceres::Problem* problem);

void BuildOptimizationProblemRCMOdo(Eigen::Vector3d* C0__t__C0_C1_estim,         // estimate of odometry as optimized
                                    Eigen::Quaternion<double>* C0__q__C1_estim,  // same as above
                                    struct config::ExperimentConfig* config,     // configuration file
                                    ceres::Problem* problem);
}  // namespace optimization
}  // namespace core

#endif  // CORE_OPTIMIZATION_PROBLEMS_H
