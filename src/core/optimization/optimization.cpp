
// region*** all includes for the optimizer (folded here)
#include "multiviperfrog/core/optimization/optimization.h"
#include <open3d/Open3D.h>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>
#include "ceres/ceres.h"
#include "glog/logging.h"
#include "multiviperfrog/config/ceres_config.h"
#include "multiviperfrog/core/optimization/cost_functions/RCM_constraint_errorterm.h"
#include "multiviperfrog/core/optimization/cost_functions/SF_knowledge_errorterm.h"
#include "multiviperfrog/core/optimization/cost_functions/Tf_and_staticlabels_from_correspondences_errorterm.h"
#include "multiviperfrog/core/optimization/cost_functions/Tf_measured_errorterm.h"
#include "multiviperfrog/core/optimization/cost_functions/data_association_errorterm.h"
#include "multiviperfrog/core/optimization/cost_functions/odo_and_trueflow_from_biasedflow_errorterm_a.h"
#include "multiviperfrog/core/optimization/cost_functions/odo_and_trueflow_from_biasedflow_errorterm_b.h"
#include "multiviperfrog/core/optimization/cost_functions/odo_from_rigidTf1_errorterm.h"
#include "multiviperfrog/core/optimization/problems.h"
#include "multiviperfrog/utils/optimization_utils.h"
#include "multiviperfrog/utils/preprocessing_utils.h"
using json = nlohmann::json;
// endregion

namespace core {
namespace optimization {

double SolveOptimizationProblem(ceres::Problem* problem, config::CeresConfig& ceres_config) {
  CHECK(problem != nullptr);
  ceres::Solver::Options options;
  options.max_num_iterations = ceres_config.max_num_iterations;  // set at 200 for miccai... but why??
  // for odo_and_labels, 1 was very far, 5 was closer, 10 was good, 20 was very good, e-4 difference btw 20 and 1000
  options.linear_solver_type = ceres_config.solver;  // TODO: test DENSE_QR !!
  //    options.linear_solver_type = ceres::DENSE_QR; //TODO: test DENSE_QR !!
  //    options.num_threads = 36; //TODO: this should not affect sparse cholesky solver, test with dense QR or dense cholesky
  //    options.dense_linear_algebra_library_type = ceres::CUDA; //TODO: this is the standard,test with LAPACK or CUDA
  ceres::Solver::Summary summary;
  ceres::Solve(options, problem, &summary);
  //        std::cout << summary.FullReport() << '\n';
  //        return summary.IsSolutionUsable();
  return summary.total_time_in_seconds;
}

double runOptimizationMICCAI(config::OptimizationData& input, config::CeresConfig& ceres_config, config::ExperimentConfig& exp_config) {
  // region*** ceres optimization (folded here)
  ceres::Problem problem;  // TODO check if this need to be passed to the function instead of being created here
  // TODO: update variables names (e.g. x1 x2 x3, m1 m2 m3, etc.)
  // cost blocks (error terms)
  // error term 0 (m0) pseudo-measure of x5 for supervision
  BuildOptimizationProblemC2CMeasured0(input.Ca0__t__Ca0_Cb0_measured,  // measure m0, pseudo-measure of x5 as superv.
                                       input.Ca0__q__Cb0_measured,      // measure m0, pseudo-measure of x5 as superv.
                                       &input.Ca0__t__Ca0_Cb0_estim,    // translation of x5 (parameter)
                                       &input.Ca0__q__Cb0_estim,        // quaternion of x5 (parameter)
                                       &exp_config,                     // config settings
                                       &problem);                       // problem

  // error term 0b (m1) pseudo-measure of x6 for supervision
  BuildOptimizationProblemC2CMeasured1(input.Ca1__t__Ca1_Cb1_measured,  // measure m1, pseudo-measure of x6 as superv.
                                       input.Ca1__q__Cb1_measured,      // measure m1, pseudo-measure of x6 as superv.
                                       &input.Ca1__t__Ca1_Cb1_estim,    // translation of x6 (parameter)
                                       &input.Ca1__q__Cb1_estim,        // quaternion of x6 (parameter)
                                       &exp_config,                     // config settings
                                       &problem);                       // problem

  // error term 1 (m1, x1, x2, x5). Odo from rigid tf  NOTE: x5 set as constant (no optimization)
  BuildOptimizationProblemRigidTf1(input.Ca1__t__Ca1_Cb1_measured,  // measure m1
                                   input.Ca1__q__Cb1_measured,      // measure m1
                                   &input.Ca0__t__Ca0_Ca1_estim,    // translation of x1 (parameter)
                                   &input.Ca0__q__Ca1_estim,        // quaternion of x1 (parameter)
                                   &input.Ca0__t__Ca0_Cb0_estim,    // translation of X5 (parameter)
                                   &input.Ca0__q__Cb0_estim,        // quaternion of X5 (parameter)
                                   &input.Cb0__t__Cb0_Cb1_estim,    // translation of x2 (parameter)
                                   &input.Cb0__q__Cb1_estim,        // quaternion of x2 (parameter)
                                   &exp_config,                     // config settings
                                   &problem);                       // problem

  // error term 2 (m2, x1, x4). Odo and true flow from biased flow Camera A.
  BuildOptimizationProblemPointsA(input.Cb0__t__Cb0_P0_measured,  // measure m5
                                  input.Ca1__t__Ca1_P1_measured,  // measure m2
                                  &input.Ca0__t__Ca0_Ca1_estim,   // translation of x1 to be optimized
                                  &input.Ca0__q__Ca1_estim,       // quaternion of x1 to be optimized
                                  &input.Cb0__t__P0_P1_estim,     // Cb0_SF (parameter)
                                  &input.Ca0__t__Ca0_Cb0_estim,   // translation of x5 (parameter)
                                  &input.Ca0__q__Cb0_estim,       // quaternion of x5 (parameter)
                                  &exp_config,                    // config settings
                                  &problem);                      // problem

  // error term 3 (m3, x2, x4). Odo and true flow from biased flow Camera B.
  BuildOptimizationProblemPointsB(input.Cb0__t__Cb0_P0_measured,  // measure m5
                                  input.Cb1__t__Cb1_P1_measured,  // measure m3
                                  &input.Cb0__t__Cb0_Cb1_estim,   // translation of x2 to be optimized
                                  &input.Cb0__q__Cb1_estim,       // quaternion of x2 to be optimized
                                  &input.Cb0__t__P0_P1_estim,     // Cb0_SF (parameter)
                                  &exp_config,                    // config settings
                                  &problem);                      // problem

  // error term 4 (m4, x5, m5). C2C0 from current points
  BuildOptimizationProblemDataAssC0(input.Ca0__t__Ca0_P0_measured,  // measure m4
                                    input.Cb0__t__Cb0_P0_measured,  // measure m5
                                    &input.Ca0__t__Ca0_Cb0_estim,   // translation of x5 (parameter)
                                    &input.Ca0__q__Cb0_estim,       // quaternion of x5 (parameter)
                                    &exp_config,                    // config settings
                                    &problem);                      // problem)

  // error term 4b (m2, x6, m3). C2C1 from flowed points
  BuildOptimizationProblemDataAssC1(input.Ca1__t__Ca1_P1_measured,  // measure m2
                                    input.Cb1__t__Cb1_P1_measured,  // measure m3
                                    &input.Ca1__t__Ca1_Cb1_estim,   // translation of x6 (parameter)
                                    &input.Ca1__q__Cb1_estim,       // quaternion of x6 (parameter)
                                    &exp_config,                    // config settings
                                    &problem);                      // problem)

  // error term 5 (to constrain Cb0_SF with some known values)
  BuildOptimizationProblemKnownSF(input.Cb0__t__P0_P1_knowledge,  // Cb0_SF knowledge
                                  input.Cb0__t__P0_P1_mask[1],    // Cb0_SF mask (1 if known, 0 if unknown)
                                  &input.Cb0__t__P0_P1_estim,     // Cb0_SF (parameter)
                                  &exp_config,                    // config settings
                                  &problem);                      // problem

  // error term 6 (m6) pseudo-measure of x1 for supervision
  BuildOptimizationProblemOdomMeasuredA(input.Ca0__t__Ca0_Ca1_measured,  // measure m6, pseudo-measure of x1 as superv.
                                        input.Ca0__q__Ca1_measured,      // measure m6, pseudo-measure of x1 as supervision
                                        &input.Ca0__t__Ca0_Ca1_estim,    // translation of x1 optimized
                                        &input.Ca0__q__Ca1_estim,        // quaternion of x1 optimized
                                        &exp_config,                     // config settings
                                        &problem);                       // problem

  // error term 7 (m7) pseudo-measure of x2 for supervision
  BuildOptimizationProblemOdomMeasuredB(input.Cb0__t__Cb0_Cb1_measured,  // measure m7, pseudo-measure of x2 as superv.
                                        input.Cb0__q__Cb1_measured,      // measure m7, pseudo-measure of x2 as supervision
                                        &input.Cb0__t__Cb0_Cb1_estim,    // translation of x2 optimized
                                        &input.Cb0__q__Cb1_estim,        // quaternion of x2 optimized
                                        &exp_config,                     // config settings
                                        &problem);                       // problem

  // solve the problem
  //    CHECK(SolveOptimizationProblem(&problem))
  //        << "The solve was not successful, exiting.";
  auto total_opt_time = SolveOptimizationProblem(&problem, ceres_config);
  return total_opt_time;
  // endregion
}

double runOptimizationTwoCameras(config::OptimizationData& input, config::CeresConfig& ceres_config, config::ExperimentConfig& exp_config) {
  // region*** ceres optimization (folded here)
  ceres::Problem problem;  // TODO check if this need to be passed to the function instead of being created here
  // TODO: update variables names (e.g. x1 x2 x3, m1 m2 m3, etc.)
  // cost blocks (error terms)

  // error term 2 (m2, x1, x4). Odo and true flow from biased flow Camera A.
  BuildOptimizationProblemPointsA(input.Cb0__t__Cb0_P0_measured,  // measure m5
                                  input.Ca1__t__Ca1_P1_measured,  // measure m2
                                  &input.Ca0__t__Ca0_Ca1_estim,   // translation of x1 to be optimized
                                  &input.Ca0__q__Ca1_estim,       // quaternion of x1 to be optimized
                                  &input.Cb0__t__P0_P1_estim,     // Cb0_SF (parameter)
                                  &input.Ca0__t__Ca0_Cb0_estim,   // translation of x5 (parameter)
                                  &input.Ca0__q__Cb0_estim,       // quaternion of x5 (parameter)
                                  &exp_config,                    // config settings
                                  &problem);                      // problem

  // error term 3 (m3, x2, x4). Odo and true flow from biased flow Camera B.
  BuildOptimizationProblemPointsB(input.Cb0__t__Cb0_P0_measured,  // measure m5
                                  input.Cb1__t__Cb1_P1_measured,  // measure m3
                                  &input.Cb0__t__Cb0_Cb1_estim,   // translation of x2 to be optimized
                                  &input.Cb0__q__Cb1_estim,       // quaternion of x2 to be optimized
                                  &input.Cb0__t__P0_P1_estim,     // Cb0_SF (parameter)
                                  &exp_config,                    // config settings
                                  &problem);                      // problem

  // error term 4 (m4, x5, m5). C2C0 from current points
  BuildOptimizationProblemDataAssC0(input.Ca0__t__Ca0_P0_measured,  // measure m4
                                    input.Cb0__t__Cb0_P0_measured,  // measure m5
                                    &input.Ca0__t__Ca0_Cb0_estim,   // translation of x5 (parameter)
                                    &input.Ca0__q__Cb0_estim,       // quaternion of x5 (parameter)
                                    &exp_config,                    // config settings
                                    &problem);                      // problem)

  // error term 4b (m2, x6, m3). C2C1 from flowed points
  BuildOptimizationProblemDataAssC1(input.Ca1__t__Ca1_P1_measured,  // measure m2
                                    input.Cb1__t__Cb1_P1_measured,  // measure m3
                                    &input.Ca1__t__Ca1_Cb1_estim,   // translation of x6 (parameter)
                                    &input.Ca1__q__Cb1_estim,       // quaternion of x6 (parameter)
                                    &exp_config,                    // config settings
                                    &problem);                      // problem)

  // error term 1 bis (x6, x1, x2, x5). all parameters, just imposing the chain
  BuildOptimizationProblemTfChain(&input.Ca0__t__Ca0_Ca1_estim,  // translation of x1 (parameter)
                                  &input.Ca0__q__Ca1_estim,      // quaternion of x1 (parameter)
                                  &input.Ca0__t__Ca0_Cb0_estim,  // translation of X5 (parameter)
                                  &input.Ca0__q__Cb0_estim,      // quaternion of X5 (parameter)
                                  &input.Cb0__t__Cb0_Cb1_estim,  // translation of x2 (parameter)
                                  &input.Cb0__q__Cb1_estim,      // quaternion of x2 (parameter)
                                  &input.Ca1__t__Ca1_Cb1_estim,  // translation of x6 (parameter)
                                  &input.Ca1__q__Cb1_estim,      // quaternion of x6 (parameter)
                                  &exp_config,                   // config settings
                                  &problem);                     // problem

  //                                    BuildOptimizationProblemTfandStaticLabels(
  //                                            input.Ca0__t__Ca0_P0_measured, // measure m5
  //                                            input.Ca1__t__Ca1_P1_measured, // measure m3
  //                                            exp_config.lambda_deformation,
  //                                            exp_config.lambda_regularization,
  //                                            &input.Ca0__t__Ca0_Ca1_estim, // translation of x2 (parameter) &Ca0__t__Ca0_Ca1_estim, //
  //                                            translation of x2 to be optimized &input.Ca0__q__Ca1_estim, // quaternion of x2 to be
  //                                            optimized &input.rigidity_labels_a_estim, // rigidity labels (1= fully rigid, 0= fully
  //                                            deformable) &exp_config, // config settings &problem); // problem
  //
  //                                    BuildOptimizationProblemTfandStaticLabels(
  //                                            input.Cb0__t__Cb0_P0_measured, // measure m5
  //                                            input.Cb1__t__Cb1_P1_measured, // measure m3
  //                                            exp_config.lambda_deformation,
  //                                            exp_config.lambda_regularization,
  //                                            &input.Cb0__t__Cb0_Cb1_estim, // translation of x2 (parameter) &Ca0__t__Ca0_Ca1_estim, //
  //                                            translation of x2 to be optimized &input.Cb0__q__Cb1_estim, // quaternion of x2 to be
  //                                            optimized &input.rigidity_labels_b_estim, // rigidity labels (1= fully rigid, 0= fully
  //                                            deformable) &exp_config, // config settings &problem); // problem

  //  BuildOptimizationProblemRCMOdo(&input.Ca0__t__Ca0_Ca1_estim,  // translation of x1 optimized
  //                                 &input.Ca0__q__Ca1_estim,      // quaternion of x1 optimized
  //                                 &exp_config,                   // config settings
  //                                 &problem);                     // problem
  //
  //  BuildOptimizationProblemRCMOdo(&input.Cb0__t__Cb0_Cb1_estim,  // translation of x2 optimized
  //                                 &input.Cb0__q__Cb1_estim,      // quaternion of x2 optimized
  //                                 &exp_config,                   // config settings
  //                                 &problem);                     // problem

// error term 5 (to constrain Cb0_SF with some known values)
  exp_config.alpha_SF = exp_config.alpha_SF_A;
  BuildOptimizationProblemKnownSF(input.Cb0__t__P0_P1_knowledge,  // Cb0_SF knowledge
                                  input.Cb0__t__P0_P1_mask[0],    // Cb0_SF mask //HARDCODED, does not work for other than 2 cameras!
                                  &input.Cb0__t__P0_P1_estim,     // Cb0_SF (parameter)
                                  &exp_config,                    // config settings
                                  &problem);                      // problem

  // error term 5 (to constrain Cb0_SF with some known values)
  exp_config.alpha_SF = exp_config.alpha_SF_B;
  BuildOptimizationProblemKnownSF(input.Cb0__t__P0_P1_knowledge,  // Cb0_SF knowledge
                                  input.Cb0__t__P0_P1_mask[1],    // Cb0_SF mask //HARDCODED, does not work for other than 2 cameras!
                                  &input.Cb0__t__P0_P1_estim,     // Cb0_SF (parameter)
                                  &exp_config,                    // config settings
                                  &problem);                      // problem

  // error term 6 (m6) pseudo-measure of x1 for supervision
  BuildOptimizationProblemOdomMeasuredA(input.Ca0__t__Ca0_Ca1_measured,  // measure m6, pseudo-measure of x1 as superv.
                                        input.Ca0__q__Ca1_measured,      // measure m6, pseudo-measure of x1 as supervision
                                        &input.Ca0__t__Ca0_Ca1_estim,    // translation of x1 optimized
                                        &input.Ca0__q__Ca1_estim,        // quaternion of x1 optimized
                                        &exp_config,                     // config settings
                                        &problem);                       // problem

  // error term 7 (m7) pseudo-measure of x2 for supervision
  BuildOptimizationProblemOdomMeasuredB(input.Cb0__t__Cb0_Cb1_measured,  // measure m7, pseudo-measure of x2 as superv.
                                        input.Cb0__q__Cb1_measured,      // measure m7, pseudo-measure of x2 as supervision
                                        &input.Cb0__t__Cb0_Cb1_estim,    // translation of x2 optimized
                                        &input.Cb0__q__Cb1_estim,        // quaternion of x2 optimized
                                        &exp_config,                     // config settings
                                        &problem);                       // problem

  //                                    // error term 0 (m0) pseudo-measure of x5 for supervision
  //                                    BuildOptimizationProblemC2CMeasured0(
  //                                            input.Ca0__t__Ca0_Cb0_measured, // measure m0, pseudo-measure of x5 as superv.
  //                                            input.Ca0__q__Cb0_measured, // measure m0, pseudo-measure of x5 as superv.
  //                                            &input.Ca0__t__Ca0_Cb0_estim, // translation of x5 (parameter)
  //                                            &input.Ca0__q__Cb0_estim, // quaternion of x5 (parameter)
  //                                            &exp_config, // config settings
  //                                            &problem); // problem

  //                                    // error term 0b (m1) pseudo-measure of x6 for supervision
  //                                    BuildOptimizationProblemC2CMeasured1(
  //                                            input.Ca1__t__Ca1_Cb1_measured, // measure m1, pseudo-measure of x6 as superv.
  //                                            input.Ca1__q__Cb1_measured, // measure m1, pseudo-measure of x6 as superv.
  //                                            &input.Ca1__t__Ca1_Cb1_estim, // translation of x6 (parameter)
  //                                            &input.Ca1__q__Cb1_estim, // quaternion of x6 (parameter)
  //                                            &exp_config, // config settings
  //                                            &problem); // problem

  //                                    // error term 1 (m1, x1, x2, x5). Odo from rigid tf  NOTE: x5 set as constant (no optimization)
  //                                    BuildOptimizationProblemRigidTf1(input.Ca1__t__Ca1_Cb1_measured, // measure m1
  //                                                                     input.Ca1__q__Cb1_measured, // measure m1
  //                                                                     &input.Ca0__t__Ca0_Ca1_estim, // translation of x1 (parameter)
  //                                                                     &input.Ca0__q__Ca1_estim, // quaternion of x1 (parameter)
  //                                                                     &input.Ca0__t__Ca0_Cb0_estim, // translation of X5 (parameter)
  //                                                                     &input.Ca0__q__Cb0_estim, // quaternion of X5 (parameter)
  //                                                                     &input.Cb0__t__Cb0_Cb1_estim, // translation of x2 (parameter)
  //                                                                     &input.Cb0__q__Cb1_estim, // quaternion of x2 (parameter)
  //                                                                     &exp_config, // config settings
  //                                                                     &problem); // problem

  // solve the problem
  auto total_opt_time = SolveOptimizationProblem(&problem, ceres_config);
  return total_opt_time;
  // endregion
}

double runOptimizationSingleOdometry(config::OptimizationData& input, config::CeresConfig& ceres_config,
                                     config::ExperimentConfig& exp_config) {
  auto start_BuildOptimizationProblem = std::chrono::high_resolution_clock::now();
  // region*** ceres optimization (folded here)
  ceres::Problem problem;  // TODO check if this need to be passed to the function instead of being created here
  // TODO: update variables names (e.g. x1 x2 x3, m1 m2 m3, etc.)
  // cost blocks (error terms)
  // Start the timer

  BuildOptimizationProblemTfandStaticLabels(
      input.Ca0__t__Ca0_P0_measured,  // measure m5
      input.Ca1__t__Ca1_P1_measured,  // measure m3
      exp_config.lambda_deformation, exp_config.lambda_regularization,
      &input.Ca0__t__Ca0_Ca1_estim,    // translation of x2 (parameter) &Ca0__t__Ca0_Ca1_estim, // translation of x2 to be optimized
      &input.Ca0__q__Ca1_estim,        // quaternion of x2 to be optimized
      &input.rigidity_labels_a_estim,  // rigidity labels (1= fully rigid, 0= fully deformable)
      &exp_config,                     // config settings
      &problem);                       // problem

//  BuildOptimizationProblemTfandStaticLabelsExplicitSF(
//      input.Ca0__t__Ca0_P0_measured,  // measure m5
//      input.Ca1__t__Ca1_P1_measured,  // measure m3
//      exp_config.lambda_deformation, exp_config.lambda_regularization,
//      &input.Ca0__t__Ca0_Ca1_estim,    // translation of x2 (parameter) &Ca0__t__Ca0_Ca1_estim, // translation of x2 to be optimized
//      &input.Ca0__q__Ca1_estim,        // quaternion of x2 to be optimized
//      &input.Cb0__t__P0_P1_estim,         // TODO change naming for use in odometry ...
//      &input.rigidity_labels_a_estim,  // rigidity labels (1= fully rigid, 0= fully deformable)
//      &exp_config,                     // config settings
//      &problem);                       // problem

//   TODO problem name for use in odometry where SF is in local frame (not always B)
  BuildOptimizationProblemPointsB(input.Ca0__t__Ca0_P0_measured, input.Ca1__t__Ca1_P1_measured, &input.Ca0__t__Ca0_Ca1_estim,
                                  &input.Ca0__q__Ca1_estim,
                                  &input.Cb0__t__P0_P1_estim,  // TODO change naming for use in odometry ...
                                  &exp_config,                 // config settings
                                  &problem);                   // problem

//  exp_config.alpha_odoA = 1000.0;
//  BuildOptimizationProblemOdomMeasuredA(input.Ca0__t__Ca0_Ca1_measured,  // measure m6, pseudo-measure of x1 as superv.
//                                        input.Ca0__q__Ca1_measured,      // measure m6, pseudo-measure of x1 as supervision
//                                        &input.Ca0__t__Ca0_Ca1_estim,    // translation of x1 optimized
//                                        &input.Ca0__q__Ca1_estim,        // quaternion of x1 optimized
//                                        &exp_config,                     // config settings
//                                        &problem);                       // problem

//    BuildOptimizationProblemPointsPlusLabels(input.Ca0__t__Ca0_P0_measured, input.Ca1__t__Ca1_P1_measured, &input.Ca0__t__Ca0_Ca1_estim,
//                                             &input.Ca0__q__Ca1_estim,
//                                             &input.Cb0__t__P0_P1_estim,         // TODO change naming for use in odometry ...
//                                             &input.rigidity_labels_a_estim,     // rigidity labels (1= fully rigid, 0= fully deformable)
//                                             &input.labels_to_SF_scaling_estim,  // scaling factor for the labels
//                                             &exp_config,                        // config settings
//                                             &problem);                          // problem

  //        BuildOptimizationProblemRCMOdo(
  //                &input.Ca0__t__Ca0_Ca1_estim, // translation of x2 optimized
  //                &input.Ca0__q__Ca1_estim, // quaternion of x2 optimized
  //                &exp_config, // config settings
  //                &problem); // problem

  // End the timer
  auto end_BuildOptimizationProblem = std::chrono::high_resolution_clock::now();
  // Calculating the time taken in seconds
  std::chrono::duration<double> diff_BuildOptimizationProblem = end_BuildOptimizationProblem - start_BuildOptimizationProblem;
  //        std::cout << "Time taken by function BuildOptimizationProblem: " << diff_BuildOptimizationProblem.count() << " seconds" <<
  //        std::endl;

  // solve the problem
  //    CHECK(SolveOptimizationProblem(&problem))
  //        << "The solve was not successful, exiting.";
  auto total_opt_time = SolveOptimizationProblem(&problem, ceres_config);
  // End the timer
  auto end_SolveOptimizationProblem = std::chrono::high_resolution_clock::now();
  // Calculating the time taken in seconds
  std::chrono::duration<double> diff_SolveOptimizationProblem = end_SolveOptimizationProblem - end_BuildOptimizationProblem;
  //        std::cout << "Time taken by function SolveOptimizationProblem: " << diff_SolveOptimizationProblem.count() << " seconds" <<
  //        std::endl;
  return total_opt_time;
  // endregion
}

}  // namespace optimization
}  // namespace core
