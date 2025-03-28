

#include "multiviperfrog/core/experiment_module.h"
#include "ceres/ceres.h"

namespace core {

void runExperimentSingleOdometry(config::CeresConfig& ceres_config, config::ExperimentConfig& exp_config,
                                 const config::PreprocessingToOptimizationData& data_from_preprocessing,
                                 config::OptimizationToPostprocessingData& data_to_postprocessing, int camera_id) {
  // Start the timer
  auto start = std::chrono::high_resolution_clock::now();
  // TODO:
  //  - move transform from o3d in preprocessing or in conversion from ros
  //  - eliminate unnecessary data copies and conversions

  // region*** GT values loading from input (folded here)
  //  transformations input
  auto Ca0__T__Ca1 = data_from_preprocessing.Cn0__T__Cn1[camera_id];

  // TODO: change the naming to remove those m and just leave the real variable names throughout!
  auto m2_pcd = data_from_preprocessing.Cn1__t__Cn1_P1_o3d_downsampled_ptr[camera_id];
  auto m4_pcd = data_from_preprocessing.Cn0__t__Cn0_P0_o3d_downsampled_ptr[camera_id];

  // define number of points in input pointcloud
  exp_config.num_points_fullcloud_downsampled = m4_pcd->points_.size();
  // important, specify this variable which is used by the Problems cost blocks!!!
  exp_config.num_points = exp_config.num_points_fullcloud_downsampled;
  std::cout << "runExperimentSingleOdometry: Num of imported points: " << exp_config.num_points << std::endl;
  // declare struct data_to_optimizer of type config::OptimizationData
  config::OptimizationData data_to_optimizer;
  // endregion
  //--------------------------------------------------------------------------------------------------------------
  // region*** GT variables conversion to Eigen (folded here)
  Eigen::Matrix4Xd Ca0__t__Ca0_P0(4, exp_config.num_points_fullcloud_downsampled);  // current points coordinates in camera 1 current
  Eigen::Matrix4Xd Ca1__t__Ca1_P1(4, exp_config.num_points_fullcloud_downsampled);  // flowed points in camera 1 next (biased)

  Ca0__t__Ca0_P0.setOnes();
  Ca1__t__Ca1_P1.setOnes();

  // Start the timer
  auto start_o3d_to_eigen = std::chrono::high_resolution_clock::now();
  for (int i = 0; i < exp_config.num_points_fullcloud_downsampled; i++) {
    Ca0__t__Ca0_P0.block(0, i, 3, 1) = m4_pcd->points_[i];
    Ca1__t__Ca1_P1.block(0, i, 3, 1) = m2_pcd->points_[i];
  }
  // End the timer
  auto end_o3d_to_eigen = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> diff_o3d_to_eigen = end_o3d_to_eigen - start_o3d_to_eigen;

  // endregion
  //--------------------------------------------------------------------------------------------------------------
  // region*** measures definition (folded here)
  //  Biased flowed points time 1 CamN
  data_to_optimizer.Ca1__t__Ca1_P1_measured = Ca1__t__Ca1_P1;
  // Current points time 0 CamN
  data_to_optimizer.Ca0__t__Ca0_P0_measured = Ca0__t__Ca0_P0;
  Eigen::Transform<double, 3, Eigen::Isometry> Ca0__T__Ca1_measured = Ca0__T__Ca1;
  data_to_optimizer.Ca0__t__Ca0_Ca1_measured = Ca0__T__Ca1_measured.translation();
  data_to_optimizer.Ca0__q__Ca1_measured = Eigen::Quaternion<double>(Ca0__T__Ca1_measured.linear());

  // endregion
  //--------------------------------------------------------------------------------------------------------------
  // region*** parameters definition (folded here)
  //  Identity initial guesses, do not edit!
  Eigen::Transform<double, 3, Eigen::Isometry> Ca0__T__Ca1_init;
  Ca0__T__Ca1_init.setIdentity();  // Trivial init = I.
  // copies of initial guesses for optimization input
  data_to_optimizer.Ca0__t__Ca0_Ca1_estim = Ca0__T__Ca1_init.translation();
  data_to_optimizer.Ca0__q__Ca1_estim = Eigen::Quaternion<double>(Ca0__T__Ca1_init.linear());
  // rigidity labels for cloud a (initialized as zeros)
  data_to_optimizer.rigidity_labels_a_estim = Eigen::VectorXd::Ones(exp_config.num_points_fullcloud_downsampled);
  data_to_optimizer.Cb0__t__P0_P1_estim.setZero(4, exp_config.num_points_fullcloud_downsampled);
  data_to_optimizer.Cb0__t__P0_P1_estim.row(3) = Eigen::VectorXd::Ones(exp_config.num_points_fullcloud_downsampled);
  data_to_optimizer.labels_to_SF_scaling_estim = 0.001;
  // endregion
  //--------------------------------------------------------------------------------------------------------------
  // region*** ceres optimization (folded here)
  //  End the timer
  auto end_input_conversion = std::chrono::high_resolution_clock::now();
  auto total_opt_time = optimization::runOptimizationSingleOdometry(data_to_optimizer, ceres_config,
                                                                    exp_config);  // TODO check if arguments are passed in the right way
  // TODO: discover why there are looding 15% of extra time...
  //  (0.23s on top of 1.46s just on measuring the time inside runOptimizationSingleOdometry vs. measuring outside of it!!!)
  // endregion
  //--------------------------------------------------------------------------------------------------------------
  // region*** output of data to postprocessing (folded here)
  // End the timer
  auto end_optimization = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> diff_input_conversion = end_input_conversion - start;
  std::chrono::duration<double> diff_optimization = end_optimization - start;
  std::chrono::duration<double> diff_only_optimization = end_optimization - end_input_conversion;
  data_to_postprocessing.Ccurrent__T__Cnext_estim[camera_id].stamp_ = data_from_preprocessing.world__T__Cn1[camera_id].stamp_;
  data_to_postprocessing.Ccurrent__T__Cnext_estim[camera_id].frame_id_ =
      data_from_preprocessing.world__T__Cn1[camera_id].frame_id_ + "_OPT";
  data_to_postprocessing.Ccurrent__T__Cnext_estim[camera_id].translation() = data_to_optimizer.Ca0__t__Ca0_Ca1_estim;
  data_to_postprocessing.Ccurrent__T__Cnext_estim[camera_id].linear() = data_to_optimizer.Ca0__q__Ca1_estim.toRotationMatrix();
  data_to_postprocessing.rigidity_labels_n_estim[camera_id] = data_to_optimizer.rigidity_labels_a_estim;
  data_to_postprocessing.Cn0__t__P0_P1_estim[camera_id] = data_to_optimizer.Cb0__t__P0_P1_estim;
  // save also the two measured downsampled clouds in Eigen format
  data_to_postprocessing.Ca0__t__Ca0_P0_measured[camera_id] = data_to_optimizer.Ca0__t__Ca0_P0_measured;
  data_to_postprocessing.Ca1__t__Ca1_P1_measured[camera_id] = data_to_optimizer.Ca1__t__Ca1_P1_measured;
  // color code the rgb values of the pointclouds based on the rigidity labels, where 1 is green, 0.5 is yellow and 0 is red, with
  // continuous scale
  data_from_preprocessing.Cn0__t__Cn0_P0_o3d_downsampled_ptr[camera_id]->colors_ =
      utils::o3dColorsVectorFromRigidityLabelsGreenRed(data_to_optimizer.rigidity_labels_a_estim);
  // endregion
  //--------------------------------------------------------------------------------------------------------------
  // region*** debug prints (folded here)
  //  End the timer
  auto end = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> diff = end - start;
  //        std::cout << "Time to convert o3d to eigen: " << diff_o3d_to_eigen.count() << "s" << std::endl;
  //        std::cout << "Total time for input conversion: " << diff_input_conversion.count() << "s" << std::endl;
  //        std::cout << "Total odometry (optimization only) time for camera " << camera_id << ": " << total_opt_time << "s" << std::endl;
  //        std::cout << "Time from end of input conversion to end of optimization: " << diff_only_optimization.count() << "s" << std::endl;
  //        std::cout << "Time from start to end of optimization: " << diff_optimization.count() << "s" << std::endl;
  std::cout << "ExperimentModule: Total odometry (optimization + I/O conversions total) time for camera " << camera_id << ": "
            << diff.count() * 1000 << "milliseconds" << std::endl;
  // print statistics of labels, the variable is an Eigen::VectorXd
  std::cout << "max label: " << data_to_optimizer.rigidity_labels_a_estim.maxCoeff() << std::endl;
  std::cout << "min label: " << data_to_optimizer.rigidity_labels_a_estim.minCoeff() << std::endl;
  std::cout << "mean label for camera " << camera_id << " : " << data_to_optimizer.rigidity_labels_a_estim.mean() << std::endl;
  // endregion
}

void runExperimentTwoCameras(config::CeresConfig& ceres_config, config::ExperimentConfig& exp_config,
                             config::PreprocessingToOptimizationData& data_from_preprocessing,
                             config::OptimizationToPostprocessingData& data_to_postprocessing, int camera_id_self, int camera_id_other) {
  // for now permutation of cameras is not allowed
  if (camera_id_self != 0 || camera_id_other != 1) {
    throw std::runtime_error(
        "runExperimentTwoCameras: camera_id_self and camera_id_other must be 0 and 1 respectively"
        "for the current implementation");
  }
  int permutation = 0;  // TODO make dynamic
  // Start the timer
  auto start = std::chrono::high_resolution_clock::now();
  // TODO:
  //  - move transform from o3d in preprocessing or in conversion from ros
  //  - eliminate unnecessary data copies and conversions

  // region*** GT values loading from input (folded here)
  //  transformations input
  auto Ca0__T__Ca1 = data_from_preprocessing.Cn0__T__Cn1[camera_id_self];
  auto Cb0__T__Cb1 = data_from_preprocessing.Cn0__T__Cn1[camera_id_other];
  // assuming camera_id_self = 0, and Cself0__T__Cother0[0] saved as Ca to Cb
  auto Ca0__T__Cb0 = data_from_preprocessing.Cself0__T__Cother0[camera_id_self];
  auto Ca1__T__Cb1 = data_from_preprocessing.Cself1__T__Cother1[camera_id_self];

  // TODO: change the naming to remove those m and just leave the real variable names throughout!
  auto m2_pcd = data_from_preprocessing.Cn1__t__Cn1_P1_o3d_overlap_downsampled_ptr[camera_id_self];
  auto m3_pcd = data_from_preprocessing.Cn1__t__Cn1_P1_o3d_overlap_downsampled_ptr[camera_id_other];
  auto m4_pcd = data_from_preprocessing.Cn0__t__Cn0_P0_o3d_overlap_downsampled_ptr[camera_id_self];
  auto m5_pcd = data_from_preprocessing.Cn0__t__Cn0_P0_o3d_overlap_downsampled_ptr[camera_id_other];

  // define number of points in input pointcloud
  exp_config.num_points_overlap_downsampled = m4_pcd->points_.size();
  // important, specify this variable which is used by the Problems cost blocks!!!
  exp_config.num_points = exp_config.num_points_overlap_downsampled;
  // initialize optimization parameters
  initializeOptimizationTwoCamerasParameters(data_from_preprocessing, exp_config);  // TODO: check if this is still needed here
  std::cout << "runExperimentTwoCameras: Num of imported points m4: " << exp_config.num_points << std::endl;
  // declare struct data_to_optimizer of type config::OptimizationData
  config::OptimizationData data_to_optimizer;
  // endregion
  //--------------------------------------------------------------------------------------------------------------
  // region*** GT variables conversion to Eigen (folded here)
  Eigen::Matrix4Xd Ca0__t__Ca0_P0(4, exp_config.num_points_overlap_downsampled);  // current points coordinates in camera 1 current
  Eigen::Matrix4Xd Ca1__t__Ca1_P1(4, exp_config.num_points_overlap_downsampled);  // flowed points in camera 1 next (biased)
  Eigen::Matrix4Xd Cb0__t__Cb0_P0(4, exp_config.num_points_overlap_downsampled);  // current points coordinates in camera 2 current
  Eigen::Matrix4Xd Cb1__t__Cb1_P1(4, exp_config.num_points_overlap_downsampled);  // flowed points in camera 2 next (biased)
  Eigen::Matrix4Xd Cb0__t__P0_P1(4, exp_config.num_points_overlap_downsampled);   // GT absolute scene flow

  Ca0__t__Ca0_P0.setOnes();
  Ca1__t__Ca1_P1.setOnes();
  Cb0__t__Cb0_P0.setOnes();
  Cb1__t__Cb1_P1.setOnes();
  Cb0__t__P0_P1.setOnes();

  // Start the timer
  auto start_o3d_to_eigen = std::chrono::high_resolution_clock::now();
  for (int i = 0; i < exp_config.num_points_overlap_downsampled; i++) {
    Ca0__t__Ca0_P0.block(0, i, 3, 1) = m4_pcd->points_[i];
    Cb0__t__Cb0_P0.block(0, i, 3, 1) = m5_pcd->points_[i];
    Ca1__t__Ca1_P1.block(0, i, 3, 1) = m2_pcd->points_[i];
    Cb1__t__Cb1_P1.block(0, i, 3, 1) = m3_pcd->points_[i];
    Cb0__t__P0_P1.block(0, i, 3, 1) = m5_pcd->normals_[i];
  }
  // End the timer
  auto end_o3d_to_eigen = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> diff_o3d_to_eigen = end_o3d_to_eigen - start_o3d_to_eigen;

  // endregion
  //--------------------------------------------------------------------------------------------------------------
  // region*** measures definition (folded here)
  //  Biased flowed points time 1 CamN
  data_to_optimizer.Ca1__t__Ca1_P1_measured = Ca1__t__Ca1_P1;
  data_to_optimizer.Cb1__t__Cb1_P1_measured = Cb1__t__Cb1_P1;
  // Current points time 0 CamN
  data_to_optimizer.Ca0__t__Ca0_P0_measured = Ca0__t__Ca0_P0;
  data_to_optimizer.Cb0__t__Cb0_P0_measured = Cb0__t__Cb0_P0;
  // take iether GT or previously optimized odometries
  Eigen::Transform<double, 3, Eigen::Isometry> Ca0__T__Ca1_measured;
  Eigen::Transform<double, 3, Eigen::Isometry> Cb0__T__Cb1_measured;

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // initialize to allocate space, needed in any case
  data_to_optimizer.Cb0__t__P0_P1_mask[camera_id_self] = data_from_preprocessing.Cn0__t__P0_P1_mask[camera_id_self];
  data_to_optimizer.Cb0__t__P0_P1_mask[camera_id_other] = data_from_preprocessing.Cn0__t__P0_P1_mask[camera_id_other];
  data_to_optimizer.Cb0__t__P0_P1_knowledge = data_from_preprocessing.Cn0__t__P0_P1_knowledge[camera_id_other];

  if (exp_config.optimize_odometries) {
    // FIRST METHOD, odometry supervision based on average of labels
    Ca0__T__Ca1_measured = data_to_postprocessing.Ccurrent__T__Cnext_estim[camera_id_self];
    Cb0__T__Cb1_measured = data_to_postprocessing.Ccurrent__T__Cnext_estim[camera_id_other];
    exp_config.alpha_odoA =
        std::pow(data_to_postprocessing.rigidity_labels_n_estim[camera_id_self].mean(), exp_config.steepness_rigidity_label_trust) *
        exp_config.ratio_rigidity_label_trust_a;
    exp_config.alpha_odoB =
        std::pow(data_to_postprocessing.rigidity_labels_n_estim[camera_id_other].mean(), exp_config.steepness_rigidity_label_trust) *
        exp_config.ratio_rigidity_label_trust_b;
    // set all normals to zero so only selected ones get plotted in rviz!
    std::fill(data_from_preprocessing.Cn0__t__Cn0_P0_o3d_overlap_downsampled_ptr[camera_id_self]->normals_.begin(),
              data_from_preprocessing.Cn0__t__Cn0_P0_o3d_overlap_downsampled_ptr[camera_id_self]->normals_.end(),
              Eigen::Vector3d(0.0, 0.0, 0.0));
    std::fill(data_from_preprocessing.Cn0__t__Cn0_P0_o3d_overlap_downsampled_ptr[camera_id_other]->normals_.begin(),
              data_from_preprocessing.Cn0__t__Cn0_P0_o3d_overlap_downsampled_ptr[camera_id_other]->normals_.end(),
              Eigen::Vector3d(0.0, 0.0, 0.0));

    // paint all points to red to show contrast with those that get selected viw percentile or threshold that will become full green
    data_from_preprocessing.Cn0__t__Cn0_P0_o3d_overlap_downsampled_ptr[camera_id_self]->PaintUniformColor(Eigen::Vector3d(1., 0., 0.));
    data_from_preprocessing.Cn0__t__Cn0_P0_o3d_overlap_downsampled_ptr[camera_id_other]->PaintUniformColor(Eigen::Vector3d(1., 0., 0.));

    // odometry supervision based on the upper percentiles of the labels or the labels higher than a threshold
    // PERCENTILE
    if (exp_config.use_absflow_GT_from_render_as_knowledge || exp_config.use_absflow_estimate_from_odometry_as_knowledge) {
      double average_upper_percentile_labels_a;
      double average_upper_percentile_labels_b;

      //      bool hardcoded_labels = false;
      //      if (exp_config.use_absflow_GT_from_render_as_knowledge && hardcoded_labels) {
      //        // set rigidity_labels_n_estim all to 1.0 for debugging purposes, of size 1000
      //        data_to_postprocessing.rigidity_labels_n_estim[camera_id_self].setOnes(1000);
      //        data_to_postprocessing.rigidity_labels_n_estim[camera_id_other].setOnes(1000);
      //        // set data_from_preprocessing.triplets_indexes_downsampled_in_overlap with  (-1, random1, random2) where random 2 is an
      //        index in
      //        // the range of data_to_postprocessing.rigidity_labels_n_estim[camera_id_self].size() and random1 is an index in the range
      //        of
      //        // data_to_optimizer.Cb0__t__P0_P1_knowledge.cols() add 100 random indexes for both
      ////        data_from_preprocessing.triplets_indexes_downsampled_in_overlap.clear();
      ////        for (int i = 0; i < 986; i++) {
      ////          data_from_preprocessing.triplets_indexes_downsampled_in_overlap.push_back(
      ////              std::make_tuple(-1, rand() % data_to_postprocessing.rigidity_labels_n_estim[camera_id_self].size(),
      ////                              rand() % data_to_optimizer.Cb0__t__P0_P1_knowledge.cols()));
      ////        }
      //      }

      std::vector<size_t> upper_percentile_indices_a =
          utils::getUpperPercentileIndices(data_to_postprocessing.rigidity_labels_n_estim[camera_id_self],
                                           average_upper_percentile_labels_a, exp_config.labels_upper_percentile);
      std::vector<size_t> upper_percentile_indices_b =
          utils::getUpperPercentileIndices(data_to_postprocessing.rigidity_labels_n_estim[camera_id_other],
                                           average_upper_percentile_labels_b, exp_config.labels_upper_percentile);
      // plot size of upper percentiles
      std::cout << "runExperimentTwoCameras: --> Size of upper percentiles of rigidity labels in overlap: "
                << upper_percentile_indices_a.size() << " and " << upper_percentile_indices_b.size() << std::endl;

      //      // print first 10 of data_from_preprocessing.triplets_indexes_downsampled_in_overlap and the size of the vector
      //      std::cout << "runExperimentTwoCameras: --> Size of triplets_indexes_downsampled_in_overlap: "
      //                << data_from_preprocessing.triplets_indexes_downsampled_in_overlap.size() << std::endl;

      // search for upper_percentile_indices in pairs_indexes_downsampled_in_overlap.first and return the pair indexs TODO fix for multiple
      // cameras
      auto found_upper_percentile_labels_a_indexes_in_overlap = utils::findMatchingIndicesInVector(
          upper_percentile_indices_a, data_from_preprocessing.downsample_indices_overlap_self[permutation]);
      auto found_upper_percentile_labels_b_indexes_in_overlap = utils::findMatchingIndicesInVector(
          upper_percentile_indices_b, data_from_preprocessing.downsample_indices_overlap_other[permutation]);

      //      auto [found_upper_percentile_labels_a_indexes_in_overlap, found_upper_percentile_labels_b_indexes_in_overlap] =
      //          utils::findMatchingTripletIndices(upper_percentile_indices_a, upper_percentile_indices_b,
      //                                            data_from_preprocessing.triplets_indexes_downsampled_in_overlap);
      // now assign the true values to Cb0__t__P0_P1_mask and the GT SF value to Cb0__t__P0_P1_knowledge
      std::vector<Eigen::Matrix4Xd> Cn0__t__P0_P1_full_frame_estim{2};
      if (!found_upper_percentile_labels_a_indexes_in_overlap.empty()) {
        std::cout << "runExperimentTwoCameras: --> Found " << found_upper_percentile_labels_a_indexes_in_overlap.size()
                  << " matching pairs for the upper percentiles of the rigidity <<a>> labels in overlap: " << std::endl;
        if (exp_config.use_absflow_GT_from_render_as_knowledge) {
          for (int i = 0; i < found_upper_percentile_labels_a_indexes_in_overlap.size(); i++) {
            int selected_index_overlap_a = found_upper_percentile_labels_a_indexes_in_overlap[i];
            if (selected_index_overlap_a >= Cb0__t__P0_P1.cols()) {
              std::cout << "runExperimentTwoCameras: --> selected overlap a index: " << selected_index_overlap_a
                        << " is out of bounds of GT knowledge vector, skipping" << std::endl;
              throw std::runtime_error("this should not happen!!");
            }
            data_to_optimizer.Cb0__t__P0_P1_mask[camera_id_self][selected_index_overlap_a] = true;
            data_to_optimizer.Cb0__t__P0_P1_knowledge.col(selected_index_overlap_a) = Cb0__t__P0_P1.col(selected_index_overlap_a);
            // color coding for rviz and debugghing
            data_from_preprocessing.Cn0__t__Cn0_P0_o3d_overlap_downsampled_ptr[camera_id_self]->colors_[selected_index_overlap_a] =
                Eigen::Vector3d(0, 1, 0);
            //            // block commented out to not overwrite labels colormap, add another pointcloud publisher if you really need to
            //            viz both! std::tuple<std::optional<int>, int, std::optional<int>> selected_triplet =
            //                data_from_preprocessing.triplets_indexes_downsampled_in_overlap[found_upper_percentile_labels_a_indexes_in_overlap[i]];
            //            auto original_downsampled_cloud_index_in_percentile = std::get<0>(selected_triplet);
            //            if (original_downsampled_cloud_index_in_percentile.has_value()) {
            //              data_from_preprocessing.Cn0__t__Cn0_P0_o3d_downsampled_ptr[camera_id_self]
            //                  ->colors_[original_downsampled_cloud_index_in_percentile.value()] = Eigen::Vector3d(0, 1, 0);
            //            } else {
            //              throw std::runtime_error("value of triplet is empty in a selected index, this should not happen by design!!");
            //            }
            //            // note this coloring below is added only for viz purposes, the Cb0__t__P0_P1_mask[camera_id_other] is not set to
            //            true
            //                        data_from_preprocessing.Cn0__t__Cn0_P0_o3d_overlap_downsampled_ptr[camera_id_other]->colors_[selected_index]
            //                        =
            //                            Eigen::Vector3d(0, 1, 0);
            // set normals of selected points to estimated abs SF value for visualization and debugging
            // here we need to change from frma of b to frame of A for visualization since the GT flow is defined in frame of B
            auto Cself__T__Cother_only_rot = data_from_preprocessing.Cself0__T__Cother0[camera_id_self];
            Cself__T__Cother_only_rot.translation().setZero();  // TODO CHECK CORRECTESS
            data_from_preprocessing.Cn0__t__Cn0_P0_o3d_overlap_downsampled_ptr[camera_id_self]->normals_[selected_index_overlap_a] =
                (Cself__T__Cother_only_rot * Cb0__t__P0_P1.col(selected_index_overlap_a)).head(3);
          }
        } else if (exp_config.use_absflow_estimate_from_odometry_as_knowledge) {
          // TODO: uncomment here if you want to compute abs SF from kinematics instead of from optimization step 1
          Cn0__t__P0_P1_full_frame_estim[camera_id_self] =
              utils::computeAbsSceneFlowFromOdometry(data_to_postprocessing.Ccurrent__T__Cnext_estim[camera_id_self],
                                                     data_to_postprocessing.Ca0__t__Ca0_P0_measured[camera_id_self],
                                                     data_to_postprocessing.Ca1__t__Ca1_P1_measured[camera_id_self]);
          // create a rotation only transform from B to A
          auto Cother__T__Cself_only_rot = data_from_preprocessing.Cself0__T__Cother0[camera_id_self].inverse();
          Cother__T__Cself_only_rot.translation().setZero();  // TODO CHECK CORRECTESS

          for (int i = 0; i < found_upper_percentile_labels_a_indexes_in_overlap.size(); i++) {
            int selected_index_overlap_a = found_upper_percentile_labels_a_indexes_in_overlap[i];
            //            if
            //            (!std::get<0>(data_from_preprocessing.triplets_indexes_downsampled_in_overlap[selected_index_overlap_a]).has_value())
            //            {
            //              throw std::runtime_error("runExperimentTwoCameras: --> selected original index a has no value, this should not
            //              happen!!");
            //            }
            auto selected_index_original_a = data_from_preprocessing.downsample_indices_overlap_self[permutation][selected_index_overlap_a];
            if (selected_index_original_a >= data_to_postprocessing.Cn0__t__P0_P1_estim[camera_id_self].cols()) {
              std::cout << "runExperimentTwoCameras: --> selected original index a: " << selected_index_original_a
                        << " is out of bounds of GT knowledge vector, skipping" << std::endl;
              throw std::runtime_error("this should not happen!!");
            }
            data_to_optimizer.Cb0__t__P0_P1_mask[camera_id_self][selected_index_overlap_a] = true;
            // here instead we transform the abs SF from frame of A to frame of B since we optimize in frame of B
            data_to_optimizer.Cb0__t__P0_P1_knowledge.col(selected_index_overlap_a) =
//                Cother__T__Cself_only_rot * data_to_postprocessing.Cn0__t__P0_P1_estim[camera_id_self].col(selected_index_original_a);
            // TODO: uncomment below if you want to assign abs SF estimated from kinematics instead of from optimization step 1
                Cother__T__Cself_only_rot* Cn0__t__P0_P1_full_frame_estim[camera_id_self].col(selected_index_original_a);
            // color coding for rviz visualization and debugging
            // set to green the points that are selected in the overlap (only those selected by the percentile, in overlap_a)
            data_from_preprocessing.Cn0__t__Cn0_P0_o3d_overlap_downsampled_ptr[camera_id_self]->colors_[selected_index_overlap_a] =
                Eigen::Vector3d(0, 1, 0);
            //            // note this coloring below is added only for viz purposes, the Cb0__t__P0_P1_mask[camera_id_other] is not set to
            //            true
            //            data_from_preprocessing.Cn0__t__Cn0_P0_o3d_overlap_downsampled_ptr[camera_id_other]->colors_[selected_index_overlap_a]
            //            =
            //                Eigen::Vector3d(0, 1, 0);
            //            // block commented out to not overwrite labels colormap, add another pointcloud publisher if you really need to
            //            viz. both!
            //            data_from_preprocessing.Cn0__t__Cn0_P0_o3d_downsampled_ptr[camera_id_self]->colors_[selected_index_original_a] =
            //                Eigen::Vector3d(0, 1, 0);
            // set normals of selected points to estimated abs SF value for visualization and debugging (in frame of A)
            data_from_preprocessing.Cn0__t__Cn0_P0_o3d_overlap_downsampled_ptr[camera_id_self]->normals_[selected_index_overlap_a] =
//                data_to_postprocessing.Cn0__t__P0_P1_estim[camera_id_self].col(selected_index_original_a).head(3);
            // TODO: uncomment below if you want to visualize abs SF estimated from kinematics instead of from optimization step 1
                Cn0__t__P0_P1_full_frame_estim[camera_id_self].col(selected_index_original_a).head(3);
          }
        }
        exp_config.alpha_SF_A =
            std::pow(average_upper_percentile_labels_a, exp_config.steepness_percentile_labels) * exp_config.ratio_percentile_labels_a;
        std::cout << "runExperimentTwoCameras: --> average_upper_percentile_labels_a: " << average_upper_percentile_labels_a << std::endl;
      } else {
        exp_config.alpha_SF_A = 0.0;
        std::cout << "runExperimentTwoCameras: --> Found 0 matching pairs for the upper percentiles "
                     "of the rigidity <<a>> labels in overlap: "
                  << std::endl;
      }
      if (!found_upper_percentile_labels_b_indexes_in_overlap.empty()) {
        std::cout << "runExperimentTwoCameras: --> Found " << found_upper_percentile_labels_b_indexes_in_overlap.size()
                  << " matching pairs for the upper percentiles of the rigidity <<b>> labels in overlap: " << std::endl;
        if (exp_config.use_absflow_GT_from_render_as_knowledge) {
          for (int i = 0; i < found_upper_percentile_labels_b_indexes_in_overlap.size(); i++) {
            int selected_index_overlap_b = found_upper_percentile_labels_b_indexes_in_overlap[i];
            if (selected_index_overlap_b >= Cb0__t__P0_P1.cols()) {
              std::cout << "runExperimentTwoCameras: --> selected overlap b index: " << selected_index_overlap_b
                        << " is out of bounds of GT knowledge vector, skipping" << std::endl;
              throw std::runtime_error("this should not happen!!");
            }
            data_to_optimizer.Cb0__t__P0_P1_mask[camera_id_other][selected_index_overlap_b] = true;
            data_to_optimizer.Cb0__t__P0_P1_knowledge.col(selected_index_overlap_b) = Cb0__t__P0_P1.col(selected_index_overlap_b);
            // color coding for rviz and debugghing
            data_from_preprocessing.Cn0__t__Cn0_P0_o3d_overlap_downsampled_ptr[camera_id_other]->colors_[selected_index_overlap_b] =
                Eigen::Vector3d(0, 1, 0);
            //            // block commented out to not overwrite labels colormap, add another pointcloud publisher if you really need to
            //            viz both! std::tuple<std::optional<int>, int, std::optional<int>> selected_triplet =
            //                data_from_preprocessing.triplets_indexes_downsampled_in_overlap[found_upper_percentile_labels_b_indexes_in_overlap[i]];
            //            auto original_downsampled_cloud_index_in_percentile = std::get<2>(selected_triplet);
            //            if (original_downsampled_cloud_index_in_percentile.has_value()) {
            //              data_from_preprocessing.Cn0__t__Cn0_P0_o3d_downsampled_ptr[camera_id_other]
            //                  ->colors_[original_downsampled_cloud_index_in_percentile.value()] = Eigen::Vector3d(0, 1, 0);
            //            } else {
            //               throw std::runtime_error("value of triplet is empty in a selected index, this should not happen by design!!");
            //            }

            //            // note this coloring below is added only for viz purposes, the Cb0__t__P0_P1_mask[camera_id_self] is not set to
            //            true
            //                        data_from_preprocessing.Cn0__t__Cn0_P0_o3d_overlap_downsampled_ptr[camera_id_self]->colors_[selected_index_overlap_b]
            //                        =
            //                            Eigen::Vector3d(0, 1, 0);
            // set normals of selected points to estimated abs SF value for visualization and debugging
            // no need to change frame for visualization since the GT flow is defined in frame of B
            data_from_preprocessing.Cn0__t__Cn0_P0_o3d_overlap_downsampled_ptr[camera_id_other]->normals_[selected_index_overlap_b] =
                Cb0__t__P0_P1.col(selected_index_overlap_b).head(3);
          }
        } else if (exp_config.use_absflow_estimate_from_odometry_as_knowledge) {
          // TODO: uncomment here if you want to compute abs SF from kinematics instead of from optimization step 1
          Cn0__t__P0_P1_full_frame_estim[camera_id_other] =
              utils::computeAbsSceneFlowFromOdometry(data_to_postprocessing.Ccurrent__T__Cnext_estim[camera_id_other],
                                                     data_to_postprocessing.Ca0__t__Ca0_P0_measured[camera_id_other],
                                                     data_to_postprocessing.Ca1__t__Ca1_P1_measured[camera_id_other]);

          for (int i = 0; i < found_upper_percentile_labels_b_indexes_in_overlap.size(); i++) {
            int selected_index_overlap_b = found_upper_percentile_labels_b_indexes_in_overlap[i];
            //            if
            //            (!std::get<2>(data_from_preprocessing.triplets_indexes_downsampled_in_overlap[selected_index_overlap_b]).has_value())
            //            {
            //              throw std::runtime_error("runExperimentTwoCameras: --> selected original index b has no value, this should not
            //              happen!!");
            //            }
            auto selected_index_original_b =
                data_from_preprocessing.downsample_indices_overlap_other[permutation][selected_index_overlap_b];
            if (selected_index_original_b >= data_to_postprocessing.Cn0__t__P0_P1_estim[camera_id_other].cols()) {
              std::cout << "runExperimentTwoCameras: --> selected overlap b index: " << selected_index_original_b
                        << " is out of bounds of estimated knowledge vector, skipping" << std::endl;
              throw std::runtime_error("this should not happen!!");
            }
            data_to_optimizer.Cb0__t__P0_P1_mask[camera_id_other][selected_index_overlap_b] = true;
            data_to_optimizer.Cb0__t__P0_P1_knowledge.col(selected_index_overlap_b) =
//                data_to_postprocessing.Cn0__t__P0_P1_estim[camera_id_other].col(selected_index_original_b);
            // TODO: uncomment below if you want to assign abs SF estimated from kinematics instead of from optimization step 1
                Cn0__t__P0_P1_full_frame_estim[camera_id_other].col(selected_index_original_b);

            // color coding for rviz visualization and debugging
            // set to green the points that are selected in the overlap (only those selected by the percentile, in overlap_b)
            data_from_preprocessing.Cn0__t__Cn0_P0_o3d_overlap_downsampled_ptr[camera_id_other]->colors_[selected_index_overlap_b] =
                Eigen::Vector3d(0, 1, 0);
            //            // note this coloring below is added only for viz purposes, the Cb0__t__P0_P1_mask[camera_id_self] is not set to
            //            true
            //            data_from_preprocessing.Cn0__t__Cn0_P0_o3d_overlap_downsampled_ptr[camera_id_self]->colors_[selected_index_overlap_b]
            //            =
            //                Eigen::Vector3d(0, 1, 0);
            //            // block commented out to not overwrite labels colormap, add another pointcloud publisher if you really need to
            //            viz. both!
            //            data_from_preprocessing.Cn0__t__Cn0_P0_o3d_downsampled_ptr[camera_id_other]->colors_[selected_index_original_b] =
            //                Eigen::Vector3d(0, 1, 0);
            // set normals of selected points to estimated abs SF value for visualization and debugging (in frame of B)
            data_from_preprocessing.Cn0__t__Cn0_P0_o3d_overlap_downsampled_ptr[camera_id_other]->normals_[selected_index_overlap_b] =
//                data_to_postprocessing.Cn0__t__P0_P1_estim[camera_id_other].col(selected_index_original_b).head(3);
            // TODO: uncomment below if you want to visualize abs SF estimated from kinematics instead of from optimization step 1
                Cn0__t__P0_P1_full_frame_estim[camera_id_other].col(selected_index_original_b).head(3);
          }
        }
        exp_config.alpha_SF_B =
            std::pow(average_upper_percentile_labels_b, exp_config.steepness_percentile_labels) * exp_config.ratio_percentile_labels_b;
        std::cout << "runExperimentTwoCameras: --> average_upper_percentile_labels_b: " << average_upper_percentile_labels_b << std::endl;
      } else {
        exp_config.alpha_SF_B = 0.0;
        std::cout << "runExperimentTwoCameras: --> Found 0 matching pairs for the upper percentiles "
                     "of the rigidity <<b>> labels in overlap: "
                  << std::endl;
      }
      // if both upper_percentile_indices a and b empty, throw message
      if (found_upper_percentile_labels_a_indexes_in_overlap.empty() && found_upper_percentile_labels_b_indexes_in_overlap.empty()) {
        std::cout << "runExperimentTwoCameras: --> No matching pairs found for the upper percentiles "
                     "of the rigidity labels in overlap"
                  << std::endl;
      }
    } else if (exp_config.set_to_zero_flow_points_with_high_label_as_knowledge) {
      // THRESHOLD
      // search for rigidity labels higher than a threshold
      std::vector<size_t> higher_than_threshold_indices_a = utils::getHigherThanThresholdIndices(
          data_to_postprocessing.rigidity_labels_n_estim[camera_id_self], exp_config.labels_higher_threshold);
      std::vector<size_t> higher_than_threshold_indices_b = utils::getHigherThanThresholdIndices(
          data_to_postprocessing.rigidity_labels_n_estim[camera_id_other], exp_config.labels_higher_threshold);
      // search for labels larger than theshold in pairs_indexes_downsampled_in_overlap.first and return the pair indexs
      auto found_larger_than_thershold_labels_a_indexes_in_overlap = utils::findMatchingIndicesInVector(
          higher_than_threshold_indices_a, data_from_preprocessing.downsample_indices_overlap_self[permutation]);
      auto found_larger_than_thershold_labels_b_indexes_in_overlap = utils::findMatchingIndicesInVector(
          higher_than_threshold_indices_b, data_from_preprocessing.downsample_indices_overlap_other[permutation]);
      //      auto [found_larger_than_thershold_labels_a_indexes_in_overlap, found_larger_than_thershold_labels_b_indexes_in_overlap] =
      //          utils::findMatchingTripletIndices(higher_than_threshold_indices_a, higher_than_threshold_indices_b,
      //                                            data_from_preprocessing.triplets_indexes_downsampled_in_overlap);
      // now assign the true values to Cb0__t__P0_P1_mask and a zero SF value to Cb0__t__P0_P1_knowledge
      // the value is hardcoded to Eigen vector of 0,0,0,1 = STATIC point
      if (!found_larger_than_thershold_labels_a_indexes_in_overlap.empty()) {
        std::cout << "runExperimentTwoCameras: --> Found " << found_larger_than_thershold_labels_a_indexes_in_overlap.size()
                  << " matching pairs for the rigidity <<a>> labels higher than the threshold in overlap: " << std::endl;
        for (int i = 0; i < found_larger_than_thershold_labels_a_indexes_in_overlap.size(); i++) {
          data_to_optimizer.Cb0__t__P0_P1_mask[camera_id_self][found_larger_than_thershold_labels_a_indexes_in_overlap[i]] = true;
          data_to_optimizer.Cb0__t__P0_P1_knowledge.col(  // TODO comment this out
              found_larger_than_thershold_labels_a_indexes_in_overlap[i]) = Eigen::Vector4d(0, 0, 0, 1);
          //          data_from_preprocessing.Cn0__t__Cn0_P0_o3d_overlap_downsampled_ptr[camera_id_self]
          //              ->colors_[found_larger_than_thershold_labels_a_indexes_in_overlap[i]] = Eigen::Vector3d(0, 1, 0);
          // one could have also used  data_from_preprocessing.Cn0__t__P0_P1_knowledge[camera_id_self]
          // .col(found_larger_than_thershold_labels_a_indexes_in_overlap[i]);
          // which is also initialized to 0,0,0,1.
        }
        exp_config.alpha_SF_A = exp_config.labels_higher_threshold * exp_config.ratio_upper_threshold_labels_a;
      } else {
        exp_config.alpha_SF_A = 0.0;
        std::cout << "runExperimentTwoCameras: --> Found 0 matching pairs for the rigidity <<a>> labels higher "
                     "than the threshold in overlap"
                  << std::endl;
      }
      if (!found_larger_than_thershold_labels_b_indexes_in_overlap.empty()) {
        std::cout << "runExperimentTwoCameras: --> Found " << found_larger_than_thershold_labels_b_indexes_in_overlap.size()
                  << " matching pairs for the rigidity <<b>> labels higher than the threshold in overlap: " << std::endl;
        for (int i = 0; i < found_larger_than_thershold_labels_b_indexes_in_overlap.size(); i++) {
          data_to_optimizer.Cb0__t__P0_P1_mask[camera_id_other][found_larger_than_thershold_labels_b_indexes_in_overlap[i]] = true;
          data_to_optimizer.Cb0__t__P0_P1_knowledge.col(  // TODO comment this out
              found_larger_than_thershold_labels_b_indexes_in_overlap[i]) = Eigen::Vector4d(0, 0, 0, 1);
          data_from_preprocessing.Cn0__t__Cn0_P0_o3d_overlap_downsampled_ptr[camera_id_other]
              ->colors_[found_larger_than_thershold_labels_b_indexes_in_overlap[i]] = Eigen::Vector3d(0, 1, 0);
        }
        exp_config.alpha_SF_B = exp_config.labels_higher_threshold * exp_config.ratio_upper_threshold_labels_b;
      } else {
        exp_config.alpha_SF_B = 0.0;
        std::cout << "runExperimentTwoCameras: --> Found 0 matching pairs for the rigidity <<b>> labels higher "
                     "than the threshold in overlap"
                  << std::endl;
      }
      // if both upper_threshold_indices empty, throw message
      if (found_larger_than_thershold_labels_a_indexes_in_overlap.empty() &&
          found_larger_than_thershold_labels_b_indexes_in_overlap.empty()) {
        std::cout << "runExperimentTwoCameras: --> No matching pairs found for the labels larger "
                     "than the threshold in overlap"
                  << std::endl;
      }
    } else if (exp_config.set_to_zero_flow_points_with_high_label_as_knowledge &&
               (exp_config.use_absflow_GT_from_render_as_knowledge || exp_config.use_absflow_estimate_from_odometry_as_knowledge)) {
      throw std::runtime_error("runExperimentTwoCameras: The combination of scene flow knowledge options is not allowed");
    } else {
      std::cout << "runExperimentTwoCameras: WARNING - No option for the knowledge of the scene flow is selected" << std::endl;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////
  } else {  // TODO make smart management of GT odometries
    Ca0__T__Ca1_measured = Ca0__T__Ca1;
    Cb0__T__Cb1_measured = Cb0__T__Cb1;
    data_to_optimizer.Cb0__t__P0_P1_knowledge =
        data_from_preprocessing.Cn0__t__P0_P1_knowledge[camera_id_other];  // equivalent here self or other
  }

  data_to_optimizer.Ca0__t__Ca0_Ca1_measured = Ca0__T__Ca1_measured.translation();
  data_to_optimizer.Ca0__q__Ca1_measured = Eigen::Quaternion<double>(Ca0__T__Ca1_measured.linear());
  data_to_optimizer.Cb0__t__Cb0_Cb1_measured = Cb0__T__Cb1_measured.translation();
  data_to_optimizer.Cb0__q__Cb1_measured = Eigen::Quaternion<double>(Cb0__T__Cb1_measured.linear());

  // endregion
  //--------------------------------------------------------------------------------------------------------------
  // region*** parameters definition (folded here)
  //  copies of initial guesses for optimization input
  data_to_optimizer.Ca0__t__Ca0_Ca1_estim = data_from_preprocessing.Cn0__T__Cn1_init[camera_id_self].translation();
  data_to_optimizer.Ca0__q__Ca1_estim = Eigen::Quaternion<double>(data_from_preprocessing.Cn0__T__Cn1_init[camera_id_self].linear());
  data_to_optimizer.Cb0__t__Cb0_Cb1_estim = data_from_preprocessing.Cn0__T__Cn1_init[camera_id_other].translation();
  data_to_optimizer.Cb0__q__Cb1_estim = Eigen::Quaternion<double>(data_from_preprocessing.Cn0__T__Cn1_init[camera_id_other].linear());
  data_to_optimizer.Ca0__t__Ca0_Cb0_estim =
      data_from_preprocessing.Cself0__T__Cother0_init[camera_id_self].translation();  // assumes Cself0__T__Cother0[0] is Ca to Cb
  data_to_optimizer.Ca0__q__Cb0_estim = Eigen::Quaternion<double>(data_from_preprocessing.Cself0__T__Cother0_init[camera_id_self].linear());
  data_to_optimizer.Ca1__t__Ca1_Cb1_estim = data_from_preprocessing.Cself1__T__Cother1_init[camera_id_self].translation();
  data_to_optimizer.Ca1__q__Cb1_estim = Eigen::Quaternion<double>(data_from_preprocessing.Cself1__T__Cother1_init[camera_id_self].linear());
  // scene flow in camera 2 frame Cb0_SF
  data_to_optimizer.Cb0__t__P0_P1_estim = data_from_preprocessing.Cn0__t__P0_P1_init[camera_id_other];
  // rigidity labels for cloud a (initialized as zeros)
  data_to_optimizer.rigidity_labels_a_estim = data_from_preprocessing.rigidity_labels_n_init[camera_id_self];
  data_to_optimizer.rigidity_labels_b_estim = data_from_preprocessing.rigidity_labels_n_init[camera_id_other];
  // endregion
  //--------------------------------------------------------------------------------------------------------------
  // region*** ceres optimization (folded here)
  //  End the timer
  auto end_input_conversion = std::chrono::high_resolution_clock::now();
  auto total_opt_time = optimization::runOptimizationTwoCameras(data_to_optimizer, ceres_config,
                                                                exp_config);  // TODO check if arguments are passed in the right way
  // End the timer
  auto end_optimization = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> diff_input_conversion = end_input_conversion - start;
  std::chrono::duration<double> diff_optimization = end_optimization - start;
  std::chrono::duration<double> diff_only_optimization = end_optimization - end_input_conversion;
  // endregion
  //--------------------------------------------------------------------------------------------------------------
  //  TODO: check if its correct to give frame_id of output_frame_ids[camera_id_self] to Cself__T__Cother
  // region*** output of data to postprocessing (folded here)
  //  setting stamps and frame ids for the output
  //  here added a new variable in case this would need to be made dynamic in the future
  std::vector<std::string> output_frame_ids{::config::max_num_cameras};
  output_frame_ids[camera_id_self] = data_from_preprocessing.world__T__Cn1[camera_id_self].frame_id_ + "_OPT";
  output_frame_ids[camera_id_other] = data_from_preprocessing.world__T__Cn1[camera_id_other].frame_id_ + "_OPT";
  std::vector<ros::Time> output_stamps_0{::config::max_num_cameras};
  output_stamps_0[camera_id_self] = data_from_preprocessing.world__T__Cn0[camera_id_self].stamp_;
  output_stamps_0[camera_id_other] = data_from_preprocessing.world__T__Cn0[camera_id_other].stamp_;
  std::vector<ros::Time> output_stamps_1{::config::max_num_cameras};
  output_stamps_1[camera_id_self] = data_from_preprocessing.world__T__Cn1[camera_id_self].stamp_;
  output_stamps_1[camera_id_other] = data_from_preprocessing.world__T__Cn1[camera_id_other].stamp_;
  // odometries
  // Cam_self
  data_to_postprocessing.Ccurrent__T__Cnext_estim[camera_id_self].stamp_ = output_stamps_1[camera_id_self];
  data_to_postprocessing.Ccurrent__T__Cnext_estim[camera_id_self].frame_id_ = output_frame_ids[camera_id_self];
  data_to_postprocessing.Ccurrent__T__Cnext_estim[camera_id_self].translation() = data_to_optimizer.Ca0__t__Ca0_Ca1_estim;
  data_to_postprocessing.Ccurrent__T__Cnext_estim[camera_id_self].linear() = data_to_optimizer.Ca0__q__Ca1_estim.toRotationMatrix();
  // Cam_other
  data_to_postprocessing.Ccurrent__T__Cnext_estim[camera_id_other].stamp_ = output_stamps_1[camera_id_other];
  data_to_postprocessing.Ccurrent__T__Cnext_estim[camera_id_other].frame_id_ = output_frame_ids[camera_id_other];
  data_to_postprocessing.Ccurrent__T__Cnext_estim[camera_id_other].translation() = data_to_optimizer.Cb0__t__Cb0_Cb1_estim;
  data_to_postprocessing.Ccurrent__T__Cnext_estim[camera_id_other].linear() = data_to_optimizer.Cb0__q__Cb1_estim.toRotationMatrix();
  // camera2camera
  // time current
  data_to_postprocessing.Cself0__T__Cother0_estim[camera_id_self].stamp_ =
      output_stamps_0[camera_id_self];  // here assuming Cself0__T__Cother0_estim[0] is Ca to Cb
  data_to_postprocessing.Cself0__T__Cother0_estim[camera_id_self].frame_id_ = output_frame_ids[camera_id_self];
  data_to_postprocessing.Cself0__T__Cother0_estim[camera_id_self].translation() = data_to_optimizer.Ca0__t__Ca0_Cb0_estim;
  data_to_postprocessing.Cself0__T__Cother0_estim[camera_id_self].linear() = data_to_optimizer.Ca0__q__Cb0_estim.toRotationMatrix();
  // time next
  data_to_postprocessing.Cself1__T__Cother1_estim[camera_id_self].stamp_ = output_stamps_1[camera_id_self];
  data_to_postprocessing.Cself1__T__Cother1_estim[camera_id_self].frame_id_ = output_frame_ids[camera_id_self];
  data_to_postprocessing.Cself1__T__Cother1_estim[camera_id_self].translation() = data_to_optimizer.Ca1__t__Ca1_Cb1_estim;
  data_to_postprocessing.Cself1__T__Cother1_estim[camera_id_self].linear() = data_to_optimizer.Ca1__q__Cb1_estim.toRotationMatrix();
  // common scene flow (saved only for camera 2, cause the common SF is defined in the frame of camera 2)
  // NOTE: in optimize_odometries output, Cn0__t__P0_P1_estim[camera_id_self] is the estimated SF for cam 1
  data_to_postprocessing.Cn0__t__P0_P1_estim[camera_id_other] = data_to_optimizer.Cb0__t__P0_P1_estim;
  //        // rigidity labels // might not be necessary here not tested might have wierd interaction with the odometry optimization
  //        data_to_postprocessing.rigidity_labels_n_estim[camera_id_self] = data_to_optimizer.rigidity_labels_a_estim;
  //        data_to_postprocessing.rigidity_labels_n_estim[camera_id_other] = data_to_optimizer.rigidity_labels_b_estim;
  // endregion
  //--------------------------------------------------------------------------------------------------------------
  // region*** debug prints (folded here)
  auto end = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> diff = end - start;
  //        std::cout << "Time to convert o3d to eigen: " << diff_o3d_to_eigen.count() << "s" << std::endl;
  //        std::cout << "Total time for input conversion: " << diff_input_conversion.count() << "s" << std::endl;
  //        std::cout << "Total multicam (optimization only) time for cameras  " << camera_id_self << " and" << camera_id_other << ": " <<
  //        total_opt_time << "s" << std::endl; std::cout << "Time from end of input conversion to end of optimization: " <<
  //        diff_only_optimization.count() << "s" << std::endl; std::cout << "Time from start to end of optimization: " <<
  //        diff_optimization.count() << "s" << std::endl;
  std::cout << "Total multicam (optimization + I/O conversions total) time for cameras " << camera_id_self << " and " << camera_id_other
            << ": " << diff.count() * 1000 << "milliseconds" << std::endl;
  // TODO add back this check when used together with odo optimization if only if labels get refined here
  // print statistics of labels, the variable is an Eigen::VectorXd
  //        std::cout << "max label a: " << data_to_optimizer.rigidity_labels_a_estim.maxCoeff() << std::endl;
  //        std::cout << "min label a: " << data_to_optimizer.rigidity_labels_a_estim.minCoeff() << std::endl;
  //        std::cout << "mean label a: " << data_to_optimizer.rigidity_labels_a_estim.mean() << std::endl;
  //        std::cout << "max label b: " << data_to_optimizer.rigidity_labels_b_estim.maxCoeff() << std::endl;
  //        std::cout << "min label b: " << data_to_optimizer.rigidity_labels_b_estim.minCoeff() << std::endl;
  //        std::cout << "mean label b: " << data_to_optimizer.rigidity_labels_b_estim.mean() << std::endl;
  // endregion
}

void runExperimentMICCAI(config::CeresConfig& ceres_config, config::ExperimentConfig& exp_config,
                         config::PreprocessingToOptimizationData& data_from_preprocessing) {
  config::output_settings output;

  // region*** GT values loading from python export - blender (folded here)
  //  transformations input
  auto Ca0__T__Cb0 = data_from_preprocessing.Cself0__T__Cother0[0];
  auto Ca1__T__Cb1 = data_from_preprocessing.Cself1__T__Cother1[0];
  auto Ca0__T__Ca1 = data_from_preprocessing.Cn0__T__Cn1[0];
  auto Cb0__T__Cb1 = data_from_preprocessing.Cn0__T__Cn1[1];

  // TODO: change the naming to remove those m and just leave the real variable names throughout!
  auto m2_pcd = data_from_preprocessing.Cn1__t__Cn1_P1_o3d_ptr[0];
  auto m2_estimated_pcd = data_from_preprocessing.Cn1__t__Cn1_P1_o3d_ptr[0];
  auto m3_pcd = data_from_preprocessing.Cn1__t__Cn1_P1_o3d_ptr[1];
  auto m3_estimated_pcd = data_from_preprocessing.Cn1__t__Cn1_P1_o3d_ptr[1];
  auto m4_pcd = data_from_preprocessing.Cn0__t__Cn0_P0_o3d_ptr[0];
  auto m5_pcd = data_from_preprocessing.Cn0__t__Cn0_P0_o3d_ptr[1];

  // define number of points in input pointcloud
  int imported_num_points = m4_pcd->points_.size();

  // declare struct data_to_optimizer of type config::OptimizationData
  config::OptimizationData data_to_optimizer;
  // endregion

  // region*** Iteration blocks (folded here)
  //  Data Associaition (DA) span iteration (here we change the noise injected in the data association before KNN search)
  for (double DA_noise_iter = exp_config.start_noise_DA; DA_noise_iter <= exp_config.finish_noise_DA;
       DA_noise_iter += exp_config.step_noise_DA) {
    exp_config.noise_DA_SD = DA_noise_iter;
    std::cout << "DA_noise_iter: " << DA_noise_iter << std::endl;
    // region*** registration, KNN search, and points sampling from input

    // transform m5 in the same reference frame of m4 using Ca0__T__Cb0_noisy
    Eigen::Transform<double, 3, Eigen::Isometry> Ca0__T__Cb0_noisy = Ca0__T__Cb0;
    utils::AddNoiseToTransform(&Ca0__T__Cb0_noisy, exp_config.noise_DA_SD, 28);
    auto m5_pcd_transformed = std::make_shared<open3d::geometry::PointCloud>();
    *m5_pcd_transformed = *m5_pcd;  // deepcopy before transform
    m5_pcd_transformed->Transform(Ca0__T__Cb0_noisy.matrix());
    Eigen::Matrix3Xd m4_eig_pcd(3, imported_num_points);
    Eigen::Matrix3Xd m5_eig_in_m4_pcd(3, imported_num_points);
    for (int i = 0; i < imported_num_points; i++) {
      m4_eig_pcd.block(0, i, 3, 1) = m4_pcd->points_[i];
      m5_eig_in_m4_pcd.block(0, i, 3, 1) = m5_pcd_transformed->points_[i];
    }

    // find the pairs of points closer than a threshold
    std::vector<std::pair<int, int>> pairs;
    utils::GetPairsOfPointsCloserThanThreshold(m4_eig_pcd, m5_eig_in_m4_pcd, exp_config.points_overlap_threshold, pairs);
    // print the number of pairs
    std::cout << "Number of pairs: " << pairs.size() << std::endl;

    // save new pointclouds with only the matched points
    auto m2_matched_pcd = std::make_shared<open3d::geometry::PointCloud>();
    auto m2_est_matched_pcd = std::make_shared<open3d::geometry::PointCloud>();
    auto m3_matched_pcd = std::make_shared<open3d::geometry::PointCloud>();
    auto m3_est_matched_pcd = std::make_shared<open3d::geometry::PointCloud>();
    auto m4_matched_pcd = std::make_shared<open3d::geometry::PointCloud>();
    auto m5_matched_pcd = std::make_shared<open3d::geometry::PointCloud>();
    for (int i = 0; i < pairs.size(); i++) {
      m2_matched_pcd->points_.push_back(m2_pcd->points_[pairs[i].first]);
      m2_est_matched_pcd->points_.push_back(m2_estimated_pcd->points_[pairs[i].first]);
      m4_matched_pcd->points_.push_back(m4_pcd->points_[pairs[i].first]);
      m4_matched_pcd->normals_.push_back(m4_pcd->normals_[pairs[i].first]);
      m3_matched_pcd->points_.push_back(m3_pcd->points_[pairs[i].second]);
      m3_est_matched_pcd->points_.push_back(m3_estimated_pcd->points_[pairs[i].second]);
      m5_matched_pcd->points_.push_back(m5_pcd->points_[pairs[i].second]);
      m5_matched_pcd->normals_.push_back(m5_pcd->normals_[pairs[i].second]);
    }
    // deepcopy the selected pointclouds
    m2_matched_pcd = std::make_shared<open3d::geometry::PointCloud>(*m2_matched_pcd);
    m2_est_matched_pcd = std::make_shared<open3d::geometry::PointCloud>(*m2_est_matched_pcd);
    m3_matched_pcd = std::make_shared<open3d::geometry::PointCloud>(*m3_matched_pcd);
    m3_est_matched_pcd = std::make_shared<open3d::geometry::PointCloud>(*m3_est_matched_pcd);
    m4_matched_pcd = std::make_shared<open3d::geometry::PointCloud>(*m4_matched_pcd);
    m5_matched_pcd = std::make_shared<open3d::geometry::PointCloud>(*m5_matched_pcd);

    std::vector<int> max_points_selection_indices;
    srand(99);  // NOTE: important to keep this seed, otherwise rand() will generate the same sequence of numbers
    for (int i = 0; i < exp_config.max_points_selection_number; i++) {
      max_points_selection_indices.push_back(rand() % pairs.size());  // TODO: check for better random number generator
                                                                      // TODO: check that indexes are not taken more than once
    }

    // save new pointclouds with the selected matched points
    auto m2_matched_selected_pcd = std::make_shared<open3d::geometry::PointCloud>();
    auto m2_est_matched_selected_pcd = std::make_shared<open3d::geometry::PointCloud>();
    auto m3_matched_selected_pcd = std::make_shared<open3d::geometry::PointCloud>();
    auto m3_est_matched_selected_pcd = std::make_shared<open3d::geometry::PointCloud>();
    auto m4_matched_selected_pcd = std::make_shared<open3d::geometry::PointCloud>();
    auto m5_matched_selected_pcd = std::make_shared<open3d::geometry::PointCloud>();
    for (int i = 0; i < exp_config.max_points_selection_number; i++) {
      m2_matched_selected_pcd->points_.push_back(m2_matched_pcd->points_[max_points_selection_indices[i]]);
      m2_est_matched_selected_pcd->points_.push_back(m2_est_matched_pcd->points_[max_points_selection_indices[i]]);
      m4_matched_selected_pcd->points_.push_back(m4_matched_pcd->points_[max_points_selection_indices[i]]);
      m4_matched_selected_pcd->normals_.push_back(m4_matched_pcd->normals_[max_points_selection_indices[i]]);
      m3_matched_selected_pcd->points_.push_back(m3_matched_pcd->points_[max_points_selection_indices[i]]);
      m3_est_matched_selected_pcd->points_.push_back(m3_est_matched_pcd->points_[max_points_selection_indices[i]]);
      m5_matched_selected_pcd->points_.push_back(m5_matched_pcd->points_[max_points_selection_indices[i]]);
      m5_matched_selected_pcd->normals_.push_back(m5_matched_pcd->normals_[max_points_selection_indices[i]]);
    }
    // deepcopy the selected pointclouds
    m2_matched_selected_pcd = std::make_shared<open3d::geometry::PointCloud>(*m2_matched_selected_pcd);
    m2_est_matched_selected_pcd = std::make_shared<open3d::geometry::PointCloud>(*m2_est_matched_selected_pcd);
    m3_matched_selected_pcd = std::make_shared<open3d::geometry::PointCloud>(*m3_matched_selected_pcd);
    m3_est_matched_selected_pcd = std::make_shared<open3d::geometry::PointCloud>(*m3_est_matched_selected_pcd);
    m4_matched_selected_pcd = std::make_shared<open3d::geometry::PointCloud>(*m4_matched_selected_pcd);
    m5_matched_selected_pcd = std::make_shared<open3d::geometry::PointCloud>(*m5_matched_selected_pcd);

    // endregion

    // selcted point number span iteration (spans only for time measure experiment, then should be fixed to specific value)
    for (int points_iter = exp_config.start_num_points; points_iter <= exp_config.finish_num_points;
         points_iter += exp_config.step_num_points) {
      exp_config.num_points = points_iter;
      std::cout << "points_iter: " << points_iter << std::endl;

      // known SF span iteration (here we change the number of known SF)
      for (int known_SF_iter = exp_config.start_num_known_SF; known_SF_iter <= exp_config.finish_num_known_SF;
           known_SF_iter += exp_config.step_num_known_SF) {
        exp_config.num_known_SF = known_SF_iter;
        std::cout << "known_SF_iter: " << known_SF_iter << std::endl;

        // noise/metric span iteration (here we change the noise level)
        // exp_config.start_noise_SD = exp_config.noise_DA_SD; //TODO comment if not needed
        // exp_config.finish_noise_SD = exp_config.noise_DA_SD; //TODO comment if not needed
        for (double noise_iter = exp_config.start_noise_SD; noise_iter <= exp_config.finish_noise_SD;
             noise_iter += exp_config.step_noise_SD) {
          exp_config.noise_SD = noise_iter;
          std::cout << "noise_iter: " << noise_iter << std::endl;

          // alpha span iteration (here we change the alpha for a specific cost block)
          for (double alpha_iter = exp_config.start_alpha; alpha_iter <= exp_config.finish_alpha; alpha_iter += exp_config.step_alpha) {
            exp_config.alpha_span = alpha_iter;
            exp_config.alpha_odoA = exp_config.alpha_span;  // TODO: NOTE: select here the cost block to be spanned (if needed)
            std::cout << "alpha_iter: " << alpha_iter << std::endl;

            // seed span iteration (here we change the seed for the random generator)
            for (int seed_iter = exp_config.start_seed; seed_iter <= exp_config.finish_seed; seed_iter++) {
              exp_config.seed = seed_iter;
              std::cout << "seed_iter: " << seed_iter << std::endl;

              // time execution iterator (spans only for time measure experiment, then should be fixed to specific value)
              double sum_total_opt_time = 0.0;
              for (int time_rep_iter = 0; time_rep_iter <= exp_config.tot_time_rep; time_rep_iter++) {
                exp_config.time_rep = time_rep_iter;
                std::cout << "time_rep_iter: " << time_rep_iter << std::endl;
                //----------------------------------------------------------------------------------------------------------------------
                //----------------------------------------------------------------------------------------------------------------------
                // region*** main iterable block (folded here)
                //----------------------------------------------------------------------------------------------------------------------
                // region*** GT variables definition (folded here)
                Eigen::Matrix4Xd Ca0__t__Ca0_P0(4, exp_config.num_points);      // current points coordinates in camera 1 current
                Eigen::Matrix4Xd Cb0__t__Cb0_P0(4, exp_config.num_points);      // current points coordinates in camera 2 current
                Eigen::Matrix4Xd Ca0__t__P0_P1(4, exp_config.num_points);       // scene flow of current points in camera 1 current
                Eigen::Matrix4Xd Cb0__t__P0_P1(4, exp_config.num_points);       // scene flow of current points in camera 2 current
                Eigen::Matrix4Xd Ca1__t__Ca1_P1(4, exp_config.num_points);      // flowed points in camera 1 next (biased)
                Eigen::Matrix4Xd Cb1__t__Cb1_P1(4, exp_config.num_points);      // flowed points in camera 2 next (biased)
                Eigen::Matrix4Xd Ca1__t__Ca1_P1_est(4, exp_config.num_points);  // estmated flowed points in camera 1 next (biased) RAFT3D
                Eigen::Matrix4Xd Cb1__t__Cb1_P1_est(4, exp_config.num_points);  // estimated flowed points in camera 2 next (biased) RAFT3D
                Ca0__t__Ca0_P0.setOnes();
                Cb0__t__Cb0_P0.setOnes();
                Ca0__t__P0_P1.setOnes();
                Cb0__t__P0_P1.setOnes();
                Ca1__t__Ca1_P1.setOnes();
                Cb1__t__Cb1_P1.setOnes();
                Ca1__t__Ca1_P1_est.setOnes();
                Cb1__t__Cb1_P1_est.setOnes();

                std::vector<int> points_subset_indices;
                utils::makePointsSubsetIndices(points_subset_indices, exp_config.num_points, exp_config.max_points_selection_number,
                                               exp_config.seed);

                auto Cb0__t__Cb0_P0_measured_o3d = std::make_shared<open3d::geometry::PointCloud>();
                auto Cb1__t__Cb1_P1_measured_o3d = std::make_shared<open3d::geometry::PointCloud>();

                for (int i = 0; i < exp_config.num_points; i++) {
                  Ca0__t__Ca0_P0.block(0, i, 3, 1) = m4_matched_selected_pcd->points_[points_subset_indices[i]];
                  Cb0__t__Cb0_P0.block(0, i, 3, 1) = m5_matched_selected_pcd->points_[points_subset_indices[i]];
                  Cb0__t__Cb0_P0_measured_o3d->points_.push_back(m5_matched_selected_pcd->points_[points_subset_indices[i]]);  // for ICP
                  Ca0__t__P0_P1.block(0, i, 3, 1) = m4_matched_selected_pcd->normals_[points_subset_indices[i]];
                  Cb0__t__P0_P1.block(0, i, 3, 1) = m5_matched_selected_pcd->normals_[points_subset_indices[i]];
                  Ca1__t__Ca1_P1.block(0, i, 3, 1) = m2_matched_selected_pcd->points_[points_subset_indices[i]];
                  Ca1__t__Ca1_P1_est.block(0, i, 3, 1) = m2_est_matched_selected_pcd->points_[points_subset_indices[i]];
                  Cb1__t__Cb1_P1_measured_o3d->points_.push_back(
                      m3_est_matched_selected_pcd->points_[points_subset_indices[i]]);  // for ICP
                  Cb1__t__Cb1_P1.block(0, i, 3, 1) = m3_matched_selected_pcd->points_[points_subset_indices[i]];
                  Cb1__t__Cb1_P1_est.block(0, i, 3, 1) = m3_est_matched_selected_pcd->points_[points_subset_indices[i]];
                }

                // endregion
                //----------------------------------------------------------------------------------------------------------------------
                // region*** Initial ADD for m2 and m3 (folded here)
                std::cout << "-----------------Initial ADD relative SF (GT vs. RAFT3D)" << std::endl;
                double ADD_metric_initial_m2_RAFT3D = 0.0;
                utils::ComputeADDMetricForSF(Ca1__t__Ca1_P1, Ca1__t__Ca1_P1_est, &ADD_metric_initial_m2_RAFT3D);
                std::cout << "ADD metric initial for m2 RAFT3D:    " << ADD_metric_initial_m2_RAFT3D << std::endl;
                double ADD_metric_initial_m3_RAFT3D = 0.0;
                utils::ComputeADDMetricForSF(Cb1__t__Cb1_P1, Cb1__t__Cb1_P1_est, &ADD_metric_initial_m3_RAFT3D);
                std::cout << "ADD metric initial for m3 RAFT3D:    " << ADD_metric_initial_m3_RAFT3D << std::endl;
                // plot m2 GT and estimated as 2 pointclouds with green and red using open3d
                auto m2_GT_pcd = std::make_shared<open3d::geometry::PointCloud>();
                auto m2_est_pcd = std::make_shared<open3d::geometry::PointCloud>();
                for (int i = 0; i < exp_config.num_points; i++) {
                  // fill pointcloud at indexes specified by points_subset_indices
                  m2_GT_pcd->points_.push_back(Ca1__t__Ca1_P1.block(0, i, 3, 1));
                  m2_est_pcd->points_.push_back(Ca1__t__Ca1_P1_est.block(0, i, 3, 1));
                }
                //                            //rearrange points inside of m2_GT_pcd given the index map in points_subset_indices
                //                            for (int i = 0; i < exp_config.num_points; i++) {
                //                                m2_GT_pcd->points_[points_subset_indices[i]] = Ca1__t__Ca1_P1.block(0, i, 3, 1);
                //                                m2_est_pcd->points_[points_subset_indices[i]] = Ca1__t__Ca1_P1_est.block(0, i, 3, 1);
                //                            }

                m2_GT_pcd->PaintUniformColor(Eigen::Vector3d(0.0, 1.0, 0.0));
                m2_est_pcd->PaintUniformColor(Eigen::Vector3d(1.0, 0.0, 0.0));
                //                            PlotCloudsCorrespondences(m2_GT_pcd, m2_est_pcd);

                // plot m3 GT and estimated as 2 pointclouds with green and red using open3d
                auto m3_GT_pcd = std::make_shared<open3d::geometry::PointCloud>();
                auto m3_est_pcd = std::make_shared<open3d::geometry::PointCloud>();
                for (int i = 0; i < exp_config.num_points; i++) {
                  m3_GT_pcd->points_.push_back(Cb1__t__Cb1_P1.block(0, i, 3, 1));
                  m3_est_pcd->points_.push_back(Cb1__t__Cb1_P1_est.block(0, i, 3, 1));
                }
                m3_GT_pcd->PaintUniformColor(Eigen::Vector3d(0.0, 1.0, 0.0));
                m3_est_pcd->PaintUniformColor(Eigen::Vector3d(1.0, 0.0, 0.0));
                //                            PlotCloudsCorrespondences(m3_GT_pcd, m3_est_pcd);
                //                            PlotCloudsCorrespondences(m2_GT_pcd, m3_GT_pcd);
                // endregion
                //----------------------------------------------------------------------------------------------------------------------
                // region*** measures definition (folded here)
                std::cout << "-----------------Input Info and Metrics-----------------" << std::endl;
                // Data Association (plus noise) input metric plot
                utils::ComputeADDMetricForTransforms(Ca0__T__Cb0, Ca0__T__Cb0_noisy, Cb0__t__Cb0_P0, &output.ADD_metric_input_DA);
                std::cout << "ADD metric input for DA: " << output.ADD_metric_input_DA << " m" << std::endl;
                // m0 (x5) C2C time 0
                Eigen::Transform<double, 3, Eigen::Isometry> Ca0__T__Cb0_measured = Ca0__T__Cb0;
                if (exp_config.alpha_C2C0 > 0.0) {
                  utils::AddNoiseToTransform(&Ca0__T__Cb0_measured, exp_config.noise_SD, exp_config.seed + 6);
                  utils::ComputeADDMetricForTransforms(Ca0__T__Cb0, Ca0__T__Cb0_measured, Cb0__t__Cb0_P0, &output.ADD_metric_input_x5);
                  std::cout << "ADD metric input for x5: " << output.ADD_metric_input_x5 << " m" << std::endl;
                }
                data_to_optimizer.Ca0__t__Ca0_Cb0_measured = Ca0__T__Cb0_measured.translation();
                data_to_optimizer.Ca0__q__Cb0_measured = Eigen::Quaternion<double>(Ca0__T__Cb0_measured.linear());
                // m1 (x6) C2C time 1
                Eigen::Transform<double, 3, Eigen::Isometry> Ca1__T__Cb1_measured = Ca1__T__Cb1;
                if (exp_config.alpha_C2C1 > 0.0 || exp_config.alpha_chain > 0.0) {
                  utils::AddNoiseToTransform(&Ca1__T__Cb1_measured, exp_config.noise_SD, exp_config.seed - 96);
                  utils::ComputeADDMetricForTransforms(Ca1__T__Cb1, Ca1__T__Cb1_measured, Cb1__t__Cb1_P1, &output.ADD_metric_input_x6);
                  std::cout << "ADD metric input for x6: " << output.ADD_metric_input_x6 << " m" << std::endl;
                }
                data_to_optimizer.Ca1__t__Ca1_Cb1_measured = Ca1__T__Cb1.translation();
                data_to_optimizer.Ca1__q__Cb1_measured = Eigen::Quaternion<double>(Ca1__T__Cb1.linear());
                // m2 Biased flowed points time 1 CamA
                data_to_optimizer.Ca1__t__Ca1_P1_measured = Ca1__t__Ca1_P1_est;
                // m3 Biased flowed points time 1 CamB
                data_to_optimizer.Cb1__t__Cb1_P1_measured = Cb1__t__Cb1_P1_est;
                // m4 current points time 0 CamA
                data_to_optimizer.Ca0__t__Ca0_P0_measured = Ca0__t__Ca0_P0;
                // m5 current points time 0 CamB
                data_to_optimizer.Cb0__t__Cb0_P0_measured = Cb0__t__Cb0_P0;
                // Cb0_SF true scene flow in CamB frame
                data_to_optimizer.Cb0__t__P0_P1_measured = Cb0__t__P0_P1;  // equal to Ca0__T__Cb0.linear().transpose() * Ca0__t__P0_P1
                if (exp_config.alpha_SF > 0.0) {
                  utils::AddNoiseToPoints(&data_to_optimizer.Cb0__t__P0_P1_measured, exp_config.noise_SD, exp_config.seed + 73);
                  utils::ComputeADDMetricForSF(Cb0__t__P0_P1, data_to_optimizer.Cb0__t__P0_P1_measured, &output.ADD_metric_input_SF);
                  std::cout << "ADD metric input for SF: " << output.ADD_metric_input_SF << std::endl;
                }
                // Cb0_SF knowledge matrix, used to show constrain-ability of the problem either via known Sflow or static Sflow
                data_to_optimizer.Cb0__t__P0_P1_knowledge =
                    data_to_optimizer.Cb0__t__P0_P1_measured;  // for now equal to true flow, could be set to zero for
                // cases in which we don't know the true flow, but we know some points are not moving
                // Cb0_SF boolean mask to select points of known true scene flow
                data_to_optimizer.Cb0__t__P0_P1_mask[0] = std::vector<bool>(exp_config.num_points,
                                                                            false);  // no points are known
                // generate num_known_SF random indices
                std::vector<int> known_indices;
                if (exp_config.num_known_SF > exp_config.num_points) {
                  std::cout << "Error: num_known_SF > num_points" << std::endl;
                  //                                            return 1; //TODO remember to enforce some safety here when moving to the
                  //                                            experiment
                }
                utils::makePointsSubsetIndices(known_indices, exp_config.num_known_SF, exp_config.num_points, exp_config.seed - 12);

                for (int i = 0; i < exp_config.num_known_SF; i++) {
                  data_to_optimizer.Cb0__t__P0_P1_mask[0][known_indices[i]] = true;
                  //                            std::cout << "Mask index: " << known_indices[i] << " set to True. (debug)" << std::endl;
                }

                //    // alternative setting, all points are known and are set to zero
                //    Cb0__t__P0_P1_knowledge.setZero(4, exp_config.num_points);
                //    Cb0__t__P0_P1_knowledge.row(3) = Eigen::VectorXd::Ones(exp_config.num_points);
                //    Cb0__t__P0_P1_mask = std::vector<bool>(exp_config.num_points, true); // all points are known
                // m6 (x1)
                Eigen::Transform<double, 3, Eigen::Isometry> Ca0__T__Ca1_measured = Ca0__T__Ca1;
                if (exp_config.alpha_odoA > 0.0) {
                  utils::AddNoiseToTransform(&Ca0__T__Ca1_measured, exp_config.noise_SD, exp_config.seed + 567);
                  utils::ComputeADDMetricForTransforms(Ca0__T__Ca1, Ca0__T__Ca1_measured, Ca1__t__Ca1_P1, &output.ADD_metric_input_x1);
                  std::cout << "ADD metric input for x1: " << output.ADD_metric_input_x1 << " m" << std::endl;
                }
                data_to_optimizer.Ca0__t__Ca0_Ca1_measured = Ca0__T__Ca1_measured.translation();  // translation of m6 same as x1
                data_to_optimizer.Ca0__q__Ca1_measured =
                    Eigen::Quaternion<double>(Ca0__T__Ca1_measured.linear());  // quaternion of m6 same as x1
                // m7 (x2)
                Eigen::Transform<double, 3, Eigen::Isometry> Cb0__T__Cb1_measured = Cb0__T__Cb1;
                if (exp_config.alpha_odoB > 0.0) {
                  utils::AddNoiseToTransform(&Cb0__T__Cb1_measured, exp_config.noise_SD, exp_config.seed - 90);
                  utils::ComputeADDMetricForTransforms(Cb0__T__Cb1, Cb0__T__Cb1_measured, Cb1__t__Cb1_P1, &output.ADD_metric_input_x2);
                  std::cout << "ADD metric input for x2: " << output.ADD_metric_input_x2 << " m" << std::endl;
                }
                data_to_optimizer.Cb0__t__Cb0_Cb1_measured = Cb0__T__Cb1_measured.translation();  // translation of m7 same as x2
                data_to_optimizer.Cb0__q__Cb1_measured =
                    Eigen::Quaternion<double>(Cb0__T__Cb1_measured.linear());  // quaternion of m7 same as x2
                // endregion
                //----------------------------------------------------------------------------------------------------------------------
                // region*** parameters definition (folded here)
                //  Identity initial guesses, do not edit!
                Eigen::Transform<double, 3, Eigen::Isometry> Ca0__T__Ca1_init;  // x1
                Eigen::Transform<double, 3, Eigen::Isometry> Cb0__T__Cb1_init;  // x2
                Ca0__T__Ca1_init.setIdentity();                                 // x1 Trivial init = I. Do not comment this line!
                Cb0__T__Cb1_init.setIdentity();                                 // x2 Trivial init = I. Do not comment this line!
                Eigen::Transform<double, 3, Eigen::Isometry> Ca0__T__Cb0_init;  // x5
                Ca0__T__Cb0_init.setIdentity();  // x5 Trivial init = I. Do not comment this and the above lines!
                Eigen::Transform<double, 3, Eigen::Isometry> Ca1__T__Cb1_init;  // x5
                Ca1__T__Cb1_init.setIdentity();  // x6 Trivial init = I. Do not comment this and the above lines!

                // set here if you want to initialize with measured values
                //    Ca0__T__Ca1_init = Ca0__T__Ca1_measured; // x1, uncomment for measured value
                //    Cb0__T__Cb1_init = Cb0__T__Cb1_measured; // x2, uncomment for measured value
                //    Ca0__T__Cb0_init = Ca0__T__Cb0_measured; // x5, uncomment for measured value
                //    Ca1__T__Cb1_init = Ca1__T__Cb1_measured; // x6, uncomment for measured value

                Eigen::Vector3d Ca0__t__Ca0_Cb0_init = Ca0__T__Cb0_init.translation();                              // x5 translation
                Eigen::Quaternion<double> Ca0__q__Cb0_init = Eigen::Quaternion<double>(Ca0__T__Cb0_init.linear());  // x5 quaternion

                Eigen::Vector3d Ca1__t__Ca1_Cb1_init = Ca1__T__Cb1_init.translation();                              // x6 translation
                Eigen::Quaternion<double> Ca1__q__Cb1_init = Eigen::Quaternion<double>(Ca1__T__Cb1_init.linear());  // x6 quaternion

                // new convention estimating SF and not flowed points
                // Ca0_SF (not used!)
                Eigen::Matrix4Xd Ca0__t__P0_P1_init;
                Ca0__t__P0_P1_init.setZero(4, data_to_optimizer.Ca0__t__Ca0_P0_measured.cols());  // init to zero, not used!
                Ca0__t__P0_P1_init.row(3) = Eigen::VectorXd::Ones(Ca0__t__P0_P1_init.cols());     // not used!
                //    // init as  x1 * m2 - m4
                //    Ca0__t__P0_P1_init = Ca0__T__Ca1_init * Ca1__t__Ca1_P1_measured - Ca0__t__Ca0_P0_measured;

                // Cb0_SF
                Eigen::Matrix4Xd Cb0__t__P0_P1_init;  // init as zero
                Cb0__t__P0_P1_init.setZero(4, exp_config.num_points);
                Cb0__t__P0_P1_init.row(3) = Eigen::VectorXd::Ones(exp_config.num_points);
                //    // init as x2 * m3 - m5 euivalent to x4-m5 in the old convention
                //    Cb0__t__P0_P1_init.block(0,0,3,exp_config.num_points) = (Cb0__T__Cb1_init *
                //    Cb1__t__Cb1_P1_measured).block(0,0,3,exp_config.num_points) -
                //                                                 Cb0__t__Cb0_P0_measured.block(0,0,3,exp_config.num_points);

                // rigidity labels for cloud a (initialized as zeros)
                Eigen::VectorXd rigidity_labels_a_init = Eigen::VectorXd::Ones(exp_config.num_points);  // TODO check initialization
                // rigidity labels for cloud b (initialized as zeros)
                Eigen::VectorXd rigidity_labels_b_init = Eigen::VectorXd::Ones(exp_config.num_points);  // TODO check initialization

                // copies of initial guesses for optimization input
                // x1
                data_to_optimizer.Ca0__t__Ca0_Ca1_estim = Ca0__T__Ca1_init.translation();  // translation of x1 to be optimized
                data_to_optimizer.Ca0__q__Ca1_estim =
                    Eigen::Quaternion<double>(Ca0__T__Ca1_init.linear());  // quaternion of x1 to be optimized
                // x2
                data_to_optimizer.Cb0__t__Cb0_Cb1_estim = Cb0__T__Cb1_init.translation();  // translation of x2 to be optimized
                data_to_optimizer.Cb0__q__Cb1_estim =
                    Eigen::Quaternion<double>(Cb0__T__Cb1_init.linear());  // quaternion of x2 to be optimized
                // Cb0_SF
                data_to_optimizer.Cb0__t__P0_P1_estim = Cb0__t__P0_P1_init;  // varibale Cb0_SF to be optimized
                // x5
                data_to_optimizer.Ca0__t__Ca0_Cb0_estim = Ca0__t__Ca0_Cb0_init;  // x5 to be optimized
                data_to_optimizer.Ca0__q__Cb0_estim = Ca0__q__Cb0_init;          // x5 to be optimized
                // x6
                data_to_optimizer.Ca1__t__Ca1_Cb1_estim = Ca1__t__Ca1_Cb1_init;  // x6 to be optimized
                data_to_optimizer.Ca1__q__Cb1_estim = Ca1__q__Cb1_init;          // x6 to be optimized
                // rigidity labels for cloud a (initialized as zeros)
                data_to_optimizer.rigidity_labels_a_estim = rigidity_labels_a_init;
                // rigidity labels for cloud b (initialized as zeros)
                data_to_optimizer.rigidity_labels_b_estim = rigidity_labels_b_init;
                // endregion
                //----------------------------------------------------------------------------------------------------------------------

                // region*** ceres optimization (folded here)
                auto total_opt_time = optimization::runOptimizationMICCAI(
                    data_to_optimizer, ceres_config, exp_config);  // TODO check if arguments are passed in the right way
                // endregion
                //----------------------------------------------------------------------------------------------------------------------
                // region*** O3D icp rigid registration (folded here)
                Eigen::Transform<double, 3, Eigen::Isometry> Cb0__T__Cb1_ICP;
                Cb0__T__Cb1_ICP.setIdentity();
                utils::ComputeICPwithO3D(Cb0__t__Cb0_P0_measured_o3d, Cb1__t__Cb1_P1_measured_o3d, &Cb0__T__Cb1_ICP, exp_config);
                // endregion
                //----------------------------------------------------------------------------------------------------------------------
                // region*** convert quaternion and translation output to Tf format (folded here)
                Eigen::Transform<double, 3, Eigen::Isometry> Ca0__T__Ca1_estim;
                Ca0__T__Ca1_estim.setIdentity();
                Ca0__T__Ca1_estim.translation() = data_to_optimizer.Ca0__t__Ca0_Ca1_estim;
                Ca0__T__Ca1_estim.linear() = data_to_optimizer.Ca0__q__Ca1_estim.toRotationMatrix();
                Eigen::Transform<double, 3, Eigen::Isometry> Cb0__T__Cb1_estim;
                Cb0__T__Cb1_estim.setIdentity();
                Cb0__T__Cb1_estim.translation() = data_to_optimizer.Cb0__t__Cb0_Cb1_estim;
                Cb0__T__Cb1_estim.linear() = data_to_optimizer.Cb0__q__Cb1_estim.toRotationMatrix();
                Eigen::Transform<double, 3, Eigen::Isometry> Ca0__T__Cb0_estim;
                Ca0__T__Cb0_estim.setIdentity();
                Ca0__T__Cb0_estim.translation() = data_to_optimizer.Ca0__t__Ca0_Cb0_estim;
                Ca0__T__Cb0_estim.linear() = data_to_optimizer.Ca0__q__Cb0_estim.toRotationMatrix();
                Eigen::Transform<double, 3, Eigen::Isometry> Ca1__T__Cb1_estim;
                Ca1__T__Cb1_estim.setIdentity();
                Ca1__T__Cb1_estim.translation() = data_to_optimizer.Ca1__t__Ca1_Cb1_estim;
                Ca1__T__Cb1_estim.linear() = data_to_optimizer.Ca1__q__Cb1_estim.toRotationMatrix();
                // endregion
                //----------------------------------------------------------------------------------------------------------------------
                // region*** print output performance (folded here)
                //     OutputOptimizedParams(Ca1__T__Cb1, // measure m1 GT
                ////    CHECK(OutputOptimizedParams(Ca1__T__Cb1, // measure m1 GT
                //                                  Ca1__t__Ca1_P1_measured, // measure m2 GT
                //                                  Cb1__t__Cb1_P1, // measure m3 GT
                //                                  Cb0__t__Cb0_P0_measured, // measure m5 GT
                //                                  Ca0__T__Ca1, // parameter x1 GT
                //                                  Ca0__T__Ca1_estim, // parameter x1
                //                                  Cb0__T__Cb1, // parameter x2 GT
                //                                  Cb0__T__Cb1_estim, // parameter x2
                //                                  Cb0__t__P0_P1, // parameter Cb0_SF GT
                //                                  Cb0__t__P0_P1_estim, // parameter Cb0_SF
                //                                  Ca0__T__Cb0, // parameter x5 GT
                //                                  Ca0__T__Cb0_estim // parameter x5
                //                                  &exp_config, // config settings
                ////                                ))
                ////        << "Error outputting optimized parameters.";
                //    );
                // endregion
                //----------------------------------------------------------------------------------------------------------------------
                // region*** compute ADD metrics (folded here)

                std::cout << "----------------------Output Metrics-------------------" << std::endl;
                //                        double ADD_metric_output_x1; // distance btw true flowed points x3_GT and x3_reconstructed by
                //                        noisy Tf output of opt.
                utils::ComputeADDMetricForTransforms(Ca0__T__Ca1,        // x1_GT
                                                     Ca0__T__Ca1_estim,  // x1_estim
                                                     Ca1__t__Ca1_P1,     // m2_GT
                                                     &output.ADD_metric_output_x1);
                std::cout << "ADD metric output for x1: " << output.ADD_metric_output_x1 << " m" << std::endl;
                //                        double ADD_metric_output_x2; // distance btw true flowed points x4_GT and x4_reconstructed by
                //                        noisy Tf output of opt.
                utils::ComputeADDMetricForTransforms(Cb0__T__Cb1,        // x2_GT
                                                     Cb0__T__Cb1_estim,  // x2_estim
                                                     Cb1__t__Cb1_P1,     // m3_GT
                                                     &output.ADD_metric_output_x2);
                std::cout << "ADD metric output for x2: " << output.ADD_metric_output_x2 << " m" << std::endl;
                utils::ComputeADDMetricForTransforms(Cb0__T__Cb1,      // x2_GT
                                                     Cb0__T__Cb1_ICP,  // x2_ICP (O3D rigid)
                                                     Cb1__t__Cb1_P1,   // m3_GT
                                                     &output.ADD_metric_output_x2_ICP);
                std::cout << "ADD metric output for x2 (O3D): " << output.ADD_metric_output_x2_ICP << " m" << std::endl;
                //                        double ADD_metric_output_x5; // distance btw current points m5_GT and m5_reconstructed by noisy Tf
                //                        output of opt.
                utils::ComputeADDMetricForTransforms(Ca0__T__Cb0,        // x5_GT
                                                     Ca0__T__Cb0_estim,  // x5_estim
                                                     Cb0__t__Cb0_P0,     // m5_GT
                                                     &output.ADD_metric_output_x5);
                std::cout << "ADD metric output for x5: " << output.ADD_metric_output_x5 << " m" << std::endl;
                //                        double ADD_metric_output_x6; // distance btw true flowed points x6_GT and x6_reconstructed by
                //                        noisy Tf output of opt.
                utils::ComputeADDMetricForTransforms(Ca1__T__Cb1,        // x6_GT
                                                     Ca1__T__Cb1_estim,  // x6_estim
                                                     Cb1__t__Cb1_P1,     // m3_GT
                                                     &output.ADD_metric_output_x6);
                std::cout << "ADD metric output for x6: " << output.ADD_metric_output_x6 << " m" << std::endl;
                //                        double ADD_metric_output_SF; // distance btw true flowed points Cb0_SF_GT and Cb0_SF_estimated by
                //                        opti.
                utils::ComputeADDMetricForSF(Cb0__t__P0_P1,                          // Cb0_SF GT
                                             data_to_optimizer.Cb0__t__P0_P1_estim,  // Cb0_SF_estim
                                             &output.ADD_metric_output_SF);
                std::cout << "ADD metric output for SF: " << output.ADD_metric_output_SF << " m" << std::endl;

                // endregion
                //----------------------------------------------------------------------------------------------------------------------
                // region*** absolute scene flow reprojection (folded here)
                //  the theory fo reprojection is Cb1__t__Cb1__P1_estim = inv(Cb0__T__Cb1_estim) * (Cb0__t__Cb0_P0 + Cb0__t__P0_P1_estim)
                //  compute it

                //                            // plot Cb0__t__Cb0_P0 and Cb0__t__Cb0_P1 with their correspondences
                //                            auto CP_GT_pcd = std::make_shared<open3d::geometry::PointCloud>();
                //                            auto CFP_GT_pcd = std::make_shared<open3d::geometry::PointCloud>();
                //                            for (int i = 0; i < exp_config.num_points; i++) {
                //                                CP_GT_pcd->points_.push_back(Cb0__t__Cb0_P0.block(0, i, 3, 1));
                //                                CFP_GT_pcd->points_.push_back(Cb0__t__Cb0_P1.block(0, i, 3, 1));
                //                            }
                //                            CP_GT_pcd->PaintUniformColor(Eigen::Vector3d(0.0, 1.0, 0.0));
                //                            CFP_GT_pcd->PaintUniformColor(Eigen::Vector3d(1.0, 0.0, 0.0));
                //                            PlotCloudsCorrespondences(CP_GT_pcd, CFP_GT_pcd);
                //
                //
                //                            // plot Cb1__t__Cb1_P0 and Cb1__t__Cb1_P1 with their correspondences
                //                            auto Cb1__t__Cb0_P0 = Cb0__T__Cb1.inverse() * Cb0__t__Cb0_P0;
                //                            auto CP_next_pcd = std::make_shared<open3d::geometry::PointCloud>();
                //                            auto CFP_next_pcd = std::make_shared<open3d::geometry::PointCloud>();
                //                            for (int i = 0; i < exp_config.num_points; i++) {
                //                                CP_next_pcd->points_.push_back(Cb1__t__Cb0_P0.block(0, i, 3, 1));
                //                                CFP_next_pcd->points_.push_back(Cb1__t__Cb1_P1.block(0, i, 3, 1));
                //                            }
                //                            CP_next_pcd->PaintUniformColor(Eigen::Vector3d(0.0, 1.0, 0.0));
                //                            CFP_next_pcd->PaintUniformColor(Eigen::Vector3d(1.0, 0.0, 0.0));
                //                            PlotCloudsCorrespondences(CP_next_pcd, CFP_next_pcd);

                //                            Cb1__t__Cb1_P1_reproj = (Cb0__t__Cb0_P0_measured + Cb0__t__P0_P1_estim);

                // compute ADD SF between Cb1__t__Cb1_P1_measured and Cb1__t__Cb1_P1_reproj
                //                            double ADD_metric_final_m3_RAFT3D = 0.0;
                //                            ComputeADDMetricForSF(Cb1__t__Cb1_P1_measured, // m3_GT or m3_RAFT3D, depending on definition
                //                                                  Cb1__t__Cb1_P1_reproj, // m3_reproj
                //                                                  &ADD_metric_final_m3_RAFT3D);
                //                            std::cout << "ADD metric final for m3 RAFT3D: " << ADD_metric_final_m3_RAFT3D << " m" <<
                //                            std::endl;

                //                             // plot Cb0__t__P0_P1 and Cb0__t__P0_P1_estim as 2 pointclouds with green and red using
                //                             open3d auto SF_GT = std::make_shared<open3d::geometry::PointCloud>(); auto SF_est =
                //                             std::make_shared<open3d::geometry::PointCloud>(); for (int i = 0; i < exp_config.num_points;
                //                             i++) {
                //                                 SF_GT->points_.push_back((Cb0__t__Cb0_P0 + Cb0__t__P0_P1).block(0, i, 3, 1));
                //                                 SF_est->points_.push_back((Cb0__t__Cb0_P0 + Cb0__t__P0_P1_estim).block(0, i, 3, 1));
                //                             }
                //                                SF_GT->PaintUniformColor(Eigen::Vector3d(0.0, 1.0, 0.0));
                //                                SF_est->PaintUniformColor(Eigen::Vector3d(1.0, 0.0, 0.0));
                //
                //                            // plot using drawgeometry
                ////                             open3d::visualization::DrawGeometries({SF_GT, SF_est});

                //                            // transform m5 with o3d using Cb0__T__Cb1
                //                            auto m5_pcd_tr = std::make_shared<open3d::geometry::PointCloud>();
                //                            *m5_pcd_tr = *m5_pcd; // deepcopy before transform
                //                            m5_pcd_tr->Transform(Cb0__T__Cb1.matrix().inverse());
                //                            open3d::visualization::DrawGeometries({m3_pcd, m5_pcd_tr});

                //                            //1 points in "current" reference frame (o3d)
                //                            auto m5_matched_selected_pcd_tr = std::make_shared<open3d::geometry::PointCloud>();
                //                            *m5_matched_selected_pcd_tr = *m5_matched_selected_pcd; // deepcopy before transform
                //
                //                            //2 add scene flow to points coordinates
                //                            for (int i = 0; i < exp_config.num_points; i++) {
                //                                m5_matched_selected_pcd_tr->points_[i] += m5_matched_selected_pcd->normals_[i];
                //                            }
                //
                //                            //3 transform points from "current" to "next" reference frame
                //                            m5_matched_selected_pcd_tr->Transform(Cb0__T__Cb1.matrix().inverse());
                //
                //                            //4 set uniform color on pointclouds and visualize
                //                            m3_matched_selected_pcd->PaintUniformColor(Eigen::Vector3d(0.0, 1.0, 0.0));
                //                            m5_matched_selected_pcd_tr->PaintUniformColor(Eigen::Vector3d(1.0, 0.0, 0.0));
                //                            open3d::visualization::DrawGeometries({m3_matched_selected_pcd, m5_matched_selected_pcd_tr});

                ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

                // vector to hold the flowed points in current frame
                Eigen::Matrix4Xd Cb0__t__Cb0_P1(4, exp_config.num_points);
                Cb0__t__Cb0_P1.setOnes();
                //                            Cb0__t__Cb0_P1 = Cb0__t__Cb0_P0_measured; // initialized with just the points
                // add scene flow to points coordinates
                for (int i = 0; i < exp_config.num_points; ++i) {
                  Cb0__t__Cb0_P1.block(0, i, 3, 1) =
                      data_to_optimizer.Cb0__t__Cb0_P0_measured.block(0, i, 3, 1) + data_to_optimizer.Cb0__t__P0_P1_estim.block(0, i, 3, 1);
                }
                // transform flowed points from "current" to "next" reference frame ("reprojection")
                Eigen::Matrix4Xd Cb1__t__Cb0_P1(4, exp_config.num_points);
                Cb1__t__Cb0_P1.setOnes();
                Cb1__t__Cb0_P1 = Cb0__T__Cb1_estim.matrix().inverse() * Cb0__t__Cb0_P1;
                // copy into o3d cloud for visualization
                auto m3_matched_selected_fwrdprojected_pcd = std::make_shared<open3d::geometry::PointCloud>();
                auto m3_matched_selected_reordered_pcd = std::make_shared<open3d::geometry::PointCloud>();
                auto m5_matched_selected_reordered_pcd = std::make_shared<open3d::geometry::PointCloud>();
                for (int i = 0; i < exp_config.num_points; i++) {
                  m3_matched_selected_fwrdprojected_pcd->points_.push_back(Cb1__t__Cb0_P1.block(0, i, 3, 1));
                  m3_matched_selected_reordered_pcd->points_.push_back(Cb1__t__Cb1_P1.block(0, i, 3, 1));
                  m5_matched_selected_reordered_pcd->points_.push_back(data_to_optimizer.Cb0__t__Cb0_P0_measured.block(0, i, 3, 1));
                }
                // set uniform color on pointclouds and visualize
                m3_matched_selected_reordered_pcd->PaintUniformColor(Eigen::Vector3d(0.0, 1.0, 0.0));
                m5_matched_selected_reordered_pcd->PaintUniformColor(Eigen::Vector3d(0.0, 0.0, 0.0));
                m3_matched_selected_fwrdprojected_pcd->PaintUniformColor(Eigen::Vector3d(0.0, 0.0, 1.0));
                //                            open3d::visualization::DrawGeometries({m3_matched_selected_pcd,
                //                            m3_matched_selected_fwrdprojected_pcd, m3_est_pcd});
                //                            PlotCloudsCorrespondencesPlusAnotherCloud(m3_matched_selected_fwrdprojected_pcd,
                //                            m3_matched_selected_reordered_pcd, m3_est_pcd);
                //                            PlotCloudsCorrespondences3clouds(m3_matched_selected_reordered_pcd,
                //                            m3_matched_selected_fwrdprojected_pcd, m3_est_pcd);
                //                            PlotCloudsCorrespondences4clouds(m5_matched_selected_reordered_pcd,
                //                            m3_matched_selected_fwrdprojected_pcd, m3_matched_selected_reordered_pcd, m3_est_pcd);

                // transform flowed points from "current" camB to "next" camA ("fwrdprojection++") //TODO: NOTE here the name m2 and m3
                // probably do not make sense anymore, revise nomenclature
                Eigen::Matrix4Xd Ca1__t__Cb0_P1(4, exp_config.num_points);
                Ca1__t__Cb0_P1.setOnes();
                Ca1__t__Cb0_P1 = Ca0__T__Ca1_estim.matrix().inverse() * Ca0__T__Cb0_estim * Cb0__t__Cb0_P1;
                // copy into o3d cloud for visualization
                auto m2_matched_selected_fwrdprojected_pcd = std::make_shared<open3d::geometry::PointCloud>();
                auto m2_matched_selected_reordered_pcd = std::make_shared<open3d::geometry::PointCloud>();
                for (int i = 0; i < exp_config.num_points; i++) {
                  m2_matched_selected_fwrdprojected_pcd->points_.push_back(Ca1__t__Cb0_P1.block(0, i, 3, 1));
                  m2_matched_selected_reordered_pcd->points_.push_back(Ca1__t__Ca1_P1.block(0, i, 3, 1));
                }
                // set uniform color on pointclouds and visualize
                m2_matched_selected_reordered_pcd->PaintUniformColor(Eigen::Vector3d(0.0, 1.0, 0.0));
                m2_matched_selected_fwrdprojected_pcd->PaintUniformColor(Eigen::Vector3d(0.0, 0.0, 1.0));
                //                            open3d::visualization::DrawGeometries({m2_matched_selected_pcd,
                //                            m2_matched_selected_fwrdprojected_pcd, m2_est_pcd}); PlotCloudsCorrespondences(m2_est_pcd,
                //                            m2_matched_selected_reordered_pcd);
                //                            PlotCloudsCorrespondencesPlusAnotherCloud(m2_matched_selected_fwrdprojected_pcd,
                //                            m2_matched_selected_reordered_pcd, m2_est_pcd);
                //                            PlotCloudsCorrespondences3clouds(m2_matched_selected_reordered_pcd,
                //                            m2_matched_selected_fwrdprojected_pcd, m2_est_pcd);
                //                            PlotCloudsCorrespondences4clouds(m5_matched_selected_reordered_pcd,
                //                            m2_matched_selected_fwrdprojected_pcd, m2_matched_selected_reordered_pcd, m2_est_pcd);

                // compute ADD SF between Ca1__t__Ca1_P1_GT and Ca1__t__Ca1_P1_reproj
                double ADD_metric_final_m2_reprojected = 0.0;
                utils::ComputeADDMetricForSF(Ca1__t__Ca1_P1,  // m2_GT
                                             Ca1__t__Cb0_P1,  // m2_reproj
                                             &ADD_metric_final_m2_reprojected);
                std::cout << "-----------------Final ADD relative SF (GT vs. Reprojected)" << std::endl;

                std::cout << "ADD metric final for m2 reprojected: " << ADD_metric_final_m2_reprojected << " m" << std::endl;

                // compute ADD SF between Cb1__t__Cb1_P1_GT and Cb1__t__Cb1_P1_reproj
                double ADD_metric_final_m3_reprojected = 0.0;
                utils::ComputeADDMetricForSF(Cb1__t__Cb1_P1,  // m3_GT
                                             Cb1__t__Cb0_P1,  // m3_reproj
                                             &ADD_metric_final_m3_reprojected);
                std::cout << "ADD metric final for m3 reprojected: " << ADD_metric_final_m3_reprojected << " m" << std::endl;

                //
                ////                             // print x2 GT and estimated
                ////                                std::cout << "Cb0__T__Cb1_GT:\n" << Cb0__T__Cb1.matrix() << std::endl;
                ////                                std::cout << "Cb0__T__Cb1_estim:\n" << Cb0__T__Cb1_estim.matrix() << std::endl;

                // endregion
                //----------------------------------------------------------------------------------------------------------------------
                // region*** rigidity labels plotting (folded here)
                //  here we visualize the original points in frame at time zero and the flowed points (measured at time 1) reprojected in
                //  frame at time zero
                // original points are blue
                // flowed points are green if deformable and black if rigid
                Eigen::Matrix4Xd Cb0__t__Cb1_P1_backprojected(
                    4, exp_config.num_points);  // TODO: NOTE here probably the nomenclature m3 does not make sense anymore, change
                Cb0__t__Cb1_P1_backprojected.setOnes();
                //                            Cb0__T__Cb1_estim.setIdentity();
                Cb0__t__Cb1_P1_backprojected = Cb0__T__Cb1_estim * Cb1__t__Cb1_P1;
                //                            Cb0__t__Cb1_P1_backprojected = Cb0__T__Cb1_ICP * Cb1__t__Cb1_P1;
                //                            Cb0__t__Cb1_P1_backprojected = Cb0__T__Cb1 * Cb1__t__Cb1_P1;
                auto m3_matched_selected_backprojected_pcd = std::make_shared<open3d::geometry::PointCloud>();
                m3_matched_selected_backprojected_pcd->colors_.resize(exp_config.num_points);
                //                            m5_matched_selected_backprojected_pcd->PaintUniformColor(Eigen::Vector3d(0.0, 0.0, 0.0));

                // set uniform color on pointclouds and visualize
                for (int i = 0; i < exp_config.num_points; i++) {
                  m3_matched_selected_backprojected_pcd->points_.push_back(Cb0__t__Cb1_P1_backprojected.block(0, i, 3, 1));
                  double color_label = data_to_optimizer.rigidity_labels_b_estim[i];
                  m3_matched_selected_backprojected_pcd->colors_[i] = Eigen::Vector3d(0.0, color_label, 0.0);
                  //                                std::cout << "label: " << i << " is: " << rigidity_labels_b_estim[i] << std::endl;
                }
                // print standard deviation of labels, the variable is an Eigen::VectorXd
                std::cout << "max label: " << data_to_optimizer.rigidity_labels_b_estim.maxCoeff() << std::endl;
                std::cout << "min label: " << data_to_optimizer.rigidity_labels_b_estim.minCoeff() << std::endl;
                std::cout << "mean label: " << data_to_optimizer.rigidity_labels_b_estim.mean() << std::endl;

                m5_matched_selected_reordered_pcd->PaintUniformColor(Eigen::Vector3d(0.9, 0.9, 0.9));
                // plots
                // flowed points with relative scene flow (uniform color)
                //                            PlotCloudsCorrespondences(m5_matched_selected_reordered_pcd,
                //                            m3_matched_selected_reordered_pcd);
                // registered flowed points with rigidity labels in color gradient
                utils::PlotCloudsCorrespondences(m5_matched_selected_reordered_pcd, m3_matched_selected_backprojected_pcd);

                //                            open3d::visualization::DrawGeometries({m5_matched_selected_backprojected_pcd});

                // endregion
                //----------------------------------------------------------------------------------------------------------------------
                // region*** debug prints (folded here)
                //     std::cout<<"--------------------Initial Guesses--------------------"<<std::endl;
                //     std::cout << "Ca0__T__Ca1_init:\n" << Ca0__T__Ca1_init.matrix() << std::endl;
                //     std::cout << "Cb0__T__Cb1_init:\n" << Cb0__T__Cb1_init.matrix() << std::endl;
                //     std::cout << "Ca0__T__Cb0_init:\n" << Ca0__T__Cb0_init.matrix() << std::endl;
                //     std::cout << "Cb0__t__P0_P1_init:\n"<<Cb0__t__P0_P1_init.col(0).transpose()<<std::endl;
                std::cout << "------------------Optimization Settings----------------" << std::endl;
                std::cout << "Total optimization time: " << total_opt_time << "s" << std::endl;
                std::cout << "Tot number of points in overlap: " << imported_num_points << std::endl;
                std::cout << "Number of selected points in overlap: " << exp_config.num_points << std::endl;
                std::cout << "Number of known True SF: " << exp_config.num_known_SF << std::endl;
                // endregion
                // endregion
                //----------------------------------------------------------------------------------------------------------------------
                //----------------------------------------------------------------------------------------------------------------------
                // add here code to average exec time (100 times) for each # of points for PLOT 1) # points vs. time
                sum_total_opt_time += total_opt_time;
              }  // time_rep_iter
              // region*** accumulate output metrics (folded here)
              output.total_avg_opt_time = sum_total_opt_time;
              output.total_avg_opt_time /= (exp_config.tot_time_rep + 1);
              output.ADD_metric_output_x1_avg += output.ADD_metric_output_x1;
              output.ADD_metric_output_x2_avg += output.ADD_metric_output_x2;
              output.ADD_metric_output_x5_avg += output.ADD_metric_output_x5;
              output.ADD_metric_output_x6_avg += output.ADD_metric_output_x6;
              output.ADD_metric_output_SF_avg += output.ADD_metric_output_SF;

              output.ADD_metric_input_DA_avg += output.ADD_metric_input_DA;
              output.ADD_metric_input_x1_avg += output.ADD_metric_input_x1;
              output.ADD_metric_input_x2_avg += output.ADD_metric_input_x2;
              output.ADD_metric_input_x5_avg += output.ADD_metric_input_x5;
              output.ADD_metric_input_x6_avg += output.ADD_metric_input_x6;
              output.ADD_metric_input_SF_avg += output.ADD_metric_input_SF;
              // endregion
            }  // seed_iter
            // region*** output metrics averaged and written to file (folded here)
            output.ADD_metric_output_x1_avg /= (exp_config.finish_seed - exp_config.start_seed + 1);
            output.ADD_metric_output_x2_avg /= (exp_config.finish_seed - exp_config.start_seed + 1);
            output.ADD_metric_output_x5_avg /= (exp_config.finish_seed - exp_config.start_seed + 1);
            output.ADD_metric_output_x6_avg /= (exp_config.finish_seed - exp_config.start_seed + 1);
            output.ADD_metric_output_SF_avg /= (exp_config.finish_seed - exp_config.start_seed + 1);

            output.ADD_metric_input_DA_avg /= (exp_config.finish_seed - exp_config.start_seed + 1);
            output.ADD_metric_input_x1_avg /= (exp_config.finish_seed - exp_config.start_seed + 1);
            output.ADD_metric_input_x2_avg /= (exp_config.finish_seed - exp_config.start_seed + 1);
            output.ADD_metric_input_x5_avg /= (exp_config.finish_seed - exp_config.start_seed + 1);
            output.ADD_metric_input_x6_avg /= (exp_config.finish_seed - exp_config.start_seed + 1);
            output.ADD_metric_input_SF_avg /= (exp_config.finish_seed - exp_config.start_seed + 1);
            if (exp_config.noise_SD == 0.0) {
              output.ADD_metric_input_x1_avg = 0.0;
              output.ADD_metric_input_x2_avg = 0.0;
              output.ADD_metric_input_x5_avg = 0.0;
              output.ADD_metric_input_x6_avg = 0.0;
              output.ADD_metric_input_SF_avg = 0.0;
            } else {
              if (output.ADD_input_rounding == 0.0) {
                // do nothing
              } else {
                double round_thr = output.ADD_input_rounding;
                output.ADD_metric_input_DA_avg = std::round(output.ADD_metric_input_DA_avg * round_thr) / round_thr;
                output.ADD_metric_input_x1_avg = std::round(output.ADD_metric_input_x1_avg * round_thr) / round_thr;
                output.ADD_metric_input_x2_avg = std::round(output.ADD_metric_input_x2_avg * round_thr) / round_thr;
                output.ADD_metric_input_x5_avg = std::round(output.ADD_metric_input_x5_avg * round_thr) / round_thr;
                output.ADD_metric_input_x6_avg = std::round(output.ADD_metric_input_x6_avg * round_thr) / round_thr;
                output.ADD_metric_input_SF_avg = std::round(output.ADD_metric_input_SF_avg * round_thr) / round_thr;
              }
            }
            if (exp_config.noise_DA_SD == 0.0) {
              output.ADD_metric_input_DA_avg = 0.0;
            } else {
              if (output.ADD_input_rounding == 0.0) {
                // do nothing
              } else {
                double round_thr = output.ADD_input_rounding;
                output.ADD_metric_input_DA_avg = std::round(output.ADD_metric_input_DA_avg * round_thr) / round_thr;
              }
            }
            utils::OutputMetricsToFile(exp_config, output);
            output.total_avg_opt_time = 0.0;
            output.ADD_metric_output_x1_avg = 0.0;
            output.ADD_metric_output_x2_avg = 0.0;
            output.ADD_metric_output_x5_avg = 0.0;
            output.ADD_metric_output_x6_avg = 0.0;
            output.ADD_metric_output_SF_avg = 0.0;
            output.ADD_metric_input_DA_avg = 0.0;
            output.ADD_metric_input_x1_avg = 0.0;
            output.ADD_metric_input_x2_avg = 0.0;
            output.ADD_metric_input_x5_avg = 0.0;
            output.ADD_metric_input_x6_avg = 0.0;
            output.ADD_metric_input_SF_avg = 0.0;
            // endregion
          }  // alpha_iter
        }    // noise_iter
      }      // known_SF_iter
    }        // points_iter
  }          // DA_iter
  // endregion
}

void initializeOptimizationTwoCamerasParameters(config::PreprocessingToOptimizationData& data_from_preprocessing,
                                                const config::ExperimentConfig& exp_config) {
  for (int i = 0; i < ::config::max_num_cameras; i++) {
    data_from_preprocessing.Cn0__T__Cn1_init[i].setIdentity();
    data_from_preprocessing.Cself0__T__Cother0_init[i].setIdentity();
    data_from_preprocessing.Cself1__T__Cother1_init[i].setIdentity();
    data_from_preprocessing.Cn0__t__P0_P1_init[i].setZero(4, exp_config.num_points_overlap_downsampled);
    data_from_preprocessing.Cn0__t__P0_P1_init[i].row(3) = Eigen::VectorXd::Ones(exp_config.num_points_overlap_downsampled);
    data_from_preprocessing.rigidity_labels_n_init[i] = Eigen::VectorXd::Ones(exp_config.num_points_overlap_downsampled);
    data_from_preprocessing.Cn0__t__P0_P1_mask[i] = std::vector<bool>(exp_config.num_points_overlap_downsampled, false);
    data_from_preprocessing.Cn0__t__P0_P1_knowledge[i] = data_from_preprocessing.Cn0__t__P0_P1_init[i];
  }
}

void setupExperiment(config::CeresConfig* config, config::ExperimentConfig* exp_config) {
  // region*** set the configuration for experiments (folded here)

  // region*** DEPRECATED Experiments
  // region*** setting for simple SINGLE test execution (no span) (folded here)
  //
  //        // experiments control variables
  //        exp_config->finish_num_points = 100; // please input the value <------ TODO: CHECK always <= m2_pcd->points_.size()
  //        exp_config->start_num_points = exp_config->finish_num_points; // TODO: CHECK value != 0 & >= start_num_known_SF;
  //        exp_config->start_num_known_SF = 5; // please input the value <-------------------
  //        exp_config->finish_num_known_SF = exp_config->start_num_known_SF; // do not change
  //        exp_config->start_noise_SD = 0.0005; // please input the value <-------------------
  //        exp_config->finish_noise_SD = exp_config->start_noise_SD; // do not change
  //        exp_config->step_noise_SD = 0.01; // do not change
  //        exp_config->start_alpha = 0.0; // please input the value <-------------------
  //        exp_config->finish_alpha = exp_config->start_alpha; // do not change
  //        exp_config->step_alpha = 0.01; // do not change
  //        exp_config->start_seed = 10; // please input the value <-------------------
  //        exp_config->finish_seed = exp_config->start_seed; // do not change
  //        exp_config->start_time_rep = 0; // (runs once) please input the value <-------------------
  //        exp_config->finish_time_rep = exp_config->start_time_rep; // do not change
  //        // cost blocks scales
  //        exp_config->alpha_pointsA = 1.0; // do not change
  //        exp_config->alpha_pointsB = 1.0; // do not change
  //        exp_config->alpha_DataAss = 1.0; // do not change
  //        exp_config->alpha_SF = 1.0; // please input the value <-------------------
  //        exp_config->alpha_odoA = 0.0; // please input the value <-------------------
  //        exp_config->alpha_odoB = 0.0; // please input the value <-------------------
  //        exp_config->alpha_C2C0 = 0.0; // please input the value <-------------------
  //        exp_config->alpha_C2C1 = 0.0; // please input the value <-------------------
  //        exp_config->output_directory = "/home/user/Documents/rosbag_python_evaluation_output/"; // please input the value
  //        <------------------- exp_config->experiment_name = "simple_test"; // please input the value <-------------------
  // endregion

  // region*** setting for plot: 1) execution_time vs. number_of_points (folded here)
  //
  //    // experiments control variables
  //    exp_config->finish_num_points = 1000; // please input the value <------ TODO: CHECK always <= m2_pcd->points_.size()
  //    exp_config->start_num_points = 1; // TODO: CHECK value != 0 & >= start_num_known_SF;
  //    exp_config->start_num_known_SF = 0; // please input the value <-------------------
  //    exp_config->finish_num_known_SF = 0; // please input the value <-------------------
  //    exp_config->start_noise_SD = 0.0005; // please input the value <-------------------
  //    exp_config->finish_noise_SD = exp_config->start_noise_SD; // do not change
  //    exp_config->step_noise_SD = 0.01; // do not change
  //    exp_config->start_alpha = 0.0; // please input the value <-------------------
  //    exp_config->finish_alpha = exp_config->start_alpha; // do not change
  //    exp_config->step_alpha = 0.01; // do not change
  //    exp_config->start_seed = 10; // please input the value <-------------------
  //    exp_config->finish_seed = exp_config->start_seed; // do not change
  //    exp_config->tot_time_rep = 0; // (n-1) please input the value <-------------------
  //    // cost blocks scales
  //    exp_config->alpha_pointsA = 1.0; // do not change
  //    exp_config->alpha_pointsB = 1.0; // do not change
  //    exp_config->alpha_DataAss = 1.0; // do not change
  //    exp_config->alpha_SF = 1.0; // please input the value <-------------------
  //    exp_config->alpha_odoA = 0.0; // please input the value <-------------------
  //    exp_config->alpha_odoB = 0.0; // please input the value <-------------------
  //    exp_config->alpha_C2C0 = 0.0; // please input the value <-------------------
  //    exp_config->alpha_C2C1 = 0.0; // please input the value <-------------------
  //    exp_config->output_directory = "/home/user/Documents/rosbag_python_evaluation_output/"; // please input the value
  //    <------------------- exp_config->experiment_name = "time_avg_test_2"; // please input the value <-------------------
  // endregion

  // region*** setting for plot: 4) # R+t input vs ADDs out
  //
  //    // experiments control variables
  //    exp_config->finish_num_points = 100; // please input the value <------ TODO: CHECK always <= m2_pcd->points_.size()
  //    exp_config->start_num_points = exp_config->finish_num_points; // TODO: CHECK value != 0 & >= start_num_known_SF;
  //    exp_config->start_num_known_SF = 0; // please input the value <-------------------
  //    exp_config->finish_num_known_SF = 0; // please input the value <-------------------
  //    exp_config->start_noise_SD = 0.0; // please input the value <-------------------
  //    exp_config->finish_noise_SD = 0.0009; // do not change
  //    exp_config->step_noise_SD = 0.00001; // do not change
  //    exp_config->start_alpha = 0.0; // please input the value <-------------------
  //    exp_config->finish_alpha = exp_config->start_alpha; // do not change
  //    exp_config->step_alpha = 0.01; // do not change
  //    exp_config->start_seed = 10; // please input the value <-------------------
  //    exp_config->finish_seed = exp_config->start_seed; // do not change
  //    exp_config->tot_time_rep = 0; // (n-1) please input the value <-------------------
  //    // cost blocks scales
  //    exp_config->alpha_pointsA = 1.0; // do not change
  //    exp_config->alpha_pointsB = 1.0; // do not change
  //    exp_config->alpha_DataAss = 1.0; // do not change
  //    exp_config->alpha_SF = 0.0; // please input the value <-------------------
  //    exp_config->alpha_odoA = 1.0; // please input the value <-------------------
  //    exp_config->alpha_odoB = 0.0; // please input the value <-------------------
  //    exp_config->alpha_C2C0 = 0.0; // please input the value <-------------------
  //    exp_config->alpha_C2C1 = 1.0; // please input the value <-------------------
  //    exp_config->output_directory = "/home/user/Documents/rosbag_python_evaluation_output/"; // please input the value
  //    <------------------- exp_config->experiment_name = "odoA_known_test"; // please input the value <-------------------
  // endregion

  // region*** setting for plot: 7) # R+t input vs ADDs out
  //
  //    // experiments control variables
  //    exp_config->finish_num_points = 100; // please input the value <------ TODO: CHECK always <= m2_pcd->points_.size()
  //    exp_config->start_num_points = exp_config->finish_num_points; // TODO: CHECK value != 0 & >= start_num_known_SF;
  //    exp_config->start_num_known_SF = 0; // please input the value <-------------------
  //    exp_config->finish_num_known_SF = 10; // please input the value <-------------------
  //    exp_config->start_noise_SD = 0.0005; // please input the value <-------------------
  //    exp_config->finish_noise_SD = 0.0005; // do not change
  //    exp_config->step_noise_SD = 0.1; // do not change
  //    exp_config->start_alpha = 0.0; // please input the value <-------------------
  //    exp_config->finish_alpha = 1.0; // do not change
  //    exp_config->step_alpha = 0.005; // do not change
  //    exp_config->start_seed = 10; // please input the value <-------------------
  //    exp_config->finish_seed = exp_config->start_seed; // do not change
  //    exp_config->tot_time_rep = 0; // (n-1) please input the value <-------------------
  //    // cost blocks scales
  //    exp_config->alpha_pointsA = 1.0; // do not change
  //    exp_config->alpha_pointsB = 1.0; // do not change
  //    exp_config->alpha_DataAss = 1.0; // do not change
  //    exp_config->alpha_SF = 100.0; // please input the value <-------------------
  //    exp_config->alpha_odoA = 1.0; // please input the value <-------------------
  //    exp_config->alpha_odoB = 0.0; // please input the value <-------------------
  //    exp_config->alpha_C2C0 = 0.0; // please input the value <-------------------
  //    exp_config->alpha_C2C1 = 1.0; // please input the value <-------------------
  //    exp_config->output_directory = "/home/user/Documents/rosbag_python_evaluation_output/"; // please input the value
  //    <------------------- exp_config->experiment_name = "combined_span_test"; // please input the value <-------------------
  // endregion

  // region*** setting for plot: x) # (ADD_SF_in vs ADDs out
  //
  //    // experiments control variables
  //    exp_config->start_noise_DA = 0.0000; // please input the value <-------------------
  //    exp_config->finish_noise_DA = 0.0000; // do not change
  //    exp_config->step_noise_DA = 0.0001; // do not change
  //    exp_config->finish_num_points = 500; // please input the value <------ TODO: CHECK always <= m2_pcd->points_.size()
  //    exp_config->start_num_points = exp_config->finish_num_points; // TODO: CHECK value != 0 & >= start_num_known_SF;
  //    exp_config->step_num_points = 10; // do not change
  //    exp_config->start_num_known_SF = 0; // please input the value <-------------------
  //    exp_config->finish_num_known_SF = 100; // please input the value <-------------------
  //    exp_config->step_num_known_SF = 10; // do not change
  //    exp_config->start_noise_SD = 0.0; // please input the value <-------------------
  //    exp_config->finish_noise_SD = 0.01; // do not change
  //    exp_config->step_noise_SD = 0.0005; // do not change
  //    exp_config->start_alpha = 1.0; // please input the value <-------------------
  //    exp_config->finish_alpha = 1.0; // do not change
  //    exp_config->step_alpha = 0.5; // do not change
  //    exp_config->start_seed = 10; // please input the value <-------------------
  //    exp_config->finish_seed = exp_config->start_seed; // do not change
  //    exp_config->tot_time_rep = 0; // (n-1) please input the value <-------------------
  //    // cost blocks scales
  //    exp_config->alpha_pointsA = 1.0; // do not change
  //    exp_config->alpha_pointsB = 1.0; // do not change
  //    exp_config->alpha_DataAssC0 = 1.0; // block estimating x5
  //    exp_config->alpha_DataAssC1 = 1.0; // block estimating x6
  //    exp_config->alpha_SF = 1.0; // please input the value <-------------------
  //    exp_config->alpha_chain = 0.0; // please input the value <-------------------
  //    exp_config->alpha_odoA = 0.0; // please input the value <-------------------
  //    exp_config->alpha_odoB = 0.0; // please input the value <-------------------
  //    exp_config->alpha_C2C0 = 0.0; // please input the value <-------------------
  //    exp_config->alpha_C2C1 = 0.0;//0.001; // please input the value <-------------------
  //    exp_config->output_directory = "/home/user/Documents/rosbag_python_evaluation_output/"; // please input the value
  //    <------------------- exp_config->experiment_name = "SF_known_+_noise_span_test"; // please input the value <-------------------
  // endregion

  // region*** setting for plot: x) # (ADD_SF_in vs ADDs out 2
  //
  //    // experiments control variables
  //    exp_config->start_noise_DA = 0.0000; // please input the value <-------------------
  //    exp_config->finish_noise_DA = 0.0000; // do not change
  //    exp_config->step_noise_DA = 0.0001; // do not change
  //    exp_config->finish_num_points = 500; // please input the value <------ TODO: CHECK always <= m2_pcd->points_.size()
  //    exp_config->start_num_points = exp_config->finish_num_points; // TODO: CHECK value != 0 & >= start_num_known_SF;
  //    exp_config->step_num_points = 10; // do not change
  //    exp_config->start_num_known_SF = 0; // please input the value <-------------------
  //    exp_config->finish_num_known_SF = 100; // please input the value <-------------------
  //    exp_config->step_num_known_SF = 10; // do not change
  //    exp_config->start_noise_SD = 0.0; // please input the value <-------------------
  //    exp_config->finish_noise_SD = 0.01; // do not change
  //    exp_config->step_noise_SD = 0.001; // do not change
  //    exp_config->start_alpha = 1.0; // please input the value <-------------------
  //    exp_config->finish_alpha = 1.0; // do not change
  //    exp_config->step_alpha = 0.5; // do not change
  //    exp_config->start_seed = 10; // please input the value <-------------------
  //    exp_config->finish_seed = exp_config->start_seed; // do not change
  //    exp_config->tot_time_rep = 0; // (n-1) please input the value <-------------------
  //    // cost blocks scales
  //    exp_config->alpha_pointsA = 1.0; // do not change
  //    exp_config->alpha_pointsB = 1.0; // do not change
  //    exp_config->alpha_DataAssC0 = 1.0; // block estimating x5
  //    exp_config->alpha_DataAssC1 = 1.0; // block estimating x6
  //    exp_config->alpha_SF = 1.0; // please input the value <-------------------
  //    exp_config->alpha_chain = 0.0; // please input the value <-------------------
  //    exp_config->alpha_odoA = 0.0; // please input the value <-------------------
  //    exp_config->alpha_odoB = 0.0; // please input the value <-------------------
  //    exp_config->alpha_C2C0 = 0.0; // please input the value <-------------------
  //    exp_config->alpha_C2C1 = 0.0;//0.001; // please input the value <-------------------
  //    exp_config->output_directory = "/home/user/Documents/rosbag_python_evaluation_output/"; // please input the value
  //    <------------------- exp_config->experiment_name = "SF_known_+_noise_span_test2"; // please input the value <-------------------
  // endregion

  // region*** setting for plot: x) # (ADD_X1_in) vs ADDs out
  //
  //    // experiments control variables
  //    exp_config->start_noise_DA = 0.0000; // please input the value <-------------------
  //    exp_config->finish_noise_DA = 0.0000; // do not change
  //    exp_config->step_noise_DA = 0.0001; // do not change
  //    exp_config->finish_num_points = 500; // please input the value <------ TODO: CHECK always <= m2_pcd->points_.size()
  //    exp_config->start_num_points = exp_config->finish_num_points; // TODO: CHECK value != 0 & >= start_num_known_SF;
  //    exp_config->step_num_points = 1; // do not change
  //    exp_config->start_num_known_SF = 0; // please input the value <-------------------
  //    exp_config->finish_num_known_SF = 0; // please input the value <-------------------
  //    exp_config->step_num_known_SF = 1; // do not change
  //    exp_config->start_noise_SD = 0.0; // please input the value <-------------------
  //    exp_config->finish_noise_SD = 0.01; // do not change
  //    exp_config->step_noise_SD = 0.0001; // do not change
  //    exp_config->start_alpha = 1.0; // please input the value <-------------------
  //    exp_config->finish_alpha = 1.0; // do not change
  //    exp_config->step_alpha = 0.5; // do not change
  //    exp_config->start_seed = 0; // please input the value <-------------------
  //    exp_config->finish_seed = 9; // do not change
  //    exp_config->tot_time_rep = 0; // (n-1) please input the value <-------------------
  //    // cost blocks scales
  //    exp_config->alpha_pointsA = 1.0; // do not change
  //    exp_config->alpha_pointsB = 1.0; // do not change
  //    exp_config->alpha_DataAssC0 = 1.0; // block estimating x5
  //    exp_config->alpha_DataAssC1 = 1.0; // block estimating x6
  //    exp_config->alpha_SF = 0.0; // please input the value <-------------------
  //    exp_config->alpha_chain = 0.0; // please input the value <-------------------
  //    exp_config->alpha_odoA = 1.0; // please input the value <-------------------
  //    exp_config->alpha_odoB = 0.0; // please input the value <-------------------
  //    exp_config->alpha_C2C0 = 0.0; // please input the value <-------------------
  //    exp_config->alpha_C2C1 = 0.0;//0.001; // please input the value <-------------------
  //    exp_config->output_directory = "/home/user/Documents/rosbag_python_evaluation_output/"; // please input the value
  //    <------------------- exp_config->experiment_name = "ODO_A_noise_span_test"; // please input the value <-------------------
  // endregion
  // endregion

  // region*** PLOTTED setting for Experiment 1A: # ADD DA input vs ADDs out
  //
  //    // experiments control variables
  //    exp_config->start_noise_DA = 0.0; // please input the value <-------------------
  //    exp_config->finish_noise_DA = 0.01; // do not change
  //    exp_config->step_noise_DA = 0.00001; // do not change
  //    exp_config->finish_num_points = 500; // please input the value <------ TODO: CHECK always <= m2_pcd->points_.size()
  //    exp_config->start_num_points = exp_config->finish_num_points; // TODO: CHECK value != 0 & >= start_num_known_SF;
  //    exp_config->step_num_points = 1; // do not change
  //    exp_config->start_num_known_SF = 0; // please input the value <-------------------
  //    exp_config->finish_num_known_SF = 0; // please input the value <-------------------
  //    exp_config->step_num_known_SF = 1; // do not change
  //    exp_config->start_noise_SD = 0.0; // please input the value <-------------------
  //    exp_config->finish_noise_SD = 0.0; // do not change
  //    exp_config->step_noise_SD = 0.1; // do not change
  //    exp_config->start_alpha = 1.0; // please input the value <-------------------
  //    exp_config->finish_alpha = 1.0; // do not change
  //    exp_config->step_alpha = 0.5; // do not change
  //    exp_config->start_seed = 0; // please input the value <-------------------
  //    exp_config->finish_seed = 9; // do not change
  //    exp_config->tot_time_rep = 0; // (n-1) please input the value <-------------------
  //    // cost blocks scales
  //    exp_config->alpha_pointsA = 1.0; // do not change
  //    exp_config->alpha_pointsB = 1.0; // do not change
  //    exp_config->alpha_DataAssC0 = 1.0; // block estimating x5
  //    exp_config->alpha_DataAssC1 = 1.0; // block estimating x6
  //    exp_config->alpha_SF = 0.0; // please input the value <-------------------
  //    exp_config->alpha_chain = 0.0; // please input the value <-------------------
  //    exp_config->alpha_odoA = 1.0; // please input the value <-------------------
  //    exp_config->alpha_odoB = 0.0; // please input the value <-------------------
  //    exp_config->alpha_C2C0 = 0.0; // please input the value <-------------------
  //    exp_config->alpha_C2C1 = 0.0;//0.001; // please input the value <-------------------
  //    exp_config->output_directory = "/home/user/Documents/rosbag_python_evaluation_output/"; // please input the value
  //    <------------------- exp_config->experiment_name = "DA_noise_span_test"; // please input the value <-------------------
  // endregion

  // region*** PLOTTED setting for Experiment 1B: # ADD DA + m3 input vs ADDs out
  //
  //    // experiments control variables
  //    exp_config->start_noise_DA = 0.0; // please input the value <-------------------
  //    exp_config->finish_noise_DA = 0.01; // do not change
  //    exp_config->step_noise_DA = 0.00001; // do not change
  //    exp_config->finish_num_points = 500; // please input the value <------ TODO: CHECK always <= m2_pcd->points_.size()
  //    exp_config->start_num_points = exp_config->finish_num_points; // TODO: CHECK value != 0 & >= start_num_known_SF;
  //    exp_config->step_num_points = 1; // do not change
  //    exp_config->start_num_known_SF = 0; // please input the value <-------------------
  //    exp_config->finish_num_known_SF = 0; // please input the value <-------------------
  //    exp_config->step_num_known_SF = 1; // do not change
  //    exp_config->start_noise_SD = 0.0; // please input the value <-------------------
  //    exp_config->finish_noise_SD = 0.0; // do not change
  //    exp_config->step_noise_SD = 0.01; // do not change
  //    exp_config->start_alpha = 1.0; // please input the value <-------------------
  //    exp_config->finish_alpha = 1.0; // do not change
  //    exp_config->step_alpha = 0.5; // do not change
  //    exp_config->start_seed = 0; // please input the value <-------------------
  //    exp_config->finish_seed = 9; // do not change
  //    exp_config->tot_time_rep = 0; // (n-1) please input the value <-------------------
  //    // cost blocks scales
  //    exp_config->alpha_pointsA = 1.0; // do not change
  //    exp_config->alpha_pointsB = 1.0; // do not change
  //    exp_config->alpha_DataAssC0 = 1.0; // block estimating x5
  //    exp_config->alpha_DataAssC1 = 1.0; // block estimating x6
  //    exp_config->alpha_SF = 0.0; // please input the value <-------------------
  //    exp_config->alpha_chain = 0.0; // please input the value <-------------------
  //    exp_config->alpha_odoA = 1.0; // please input the value <-------------------
  //    exp_config->alpha_odoB = 0.0; // please input the value <-------------------
  //    exp_config->alpha_C2C0 = 0.0; // please input the value <-------------------
  //    exp_config->alpha_C2C1 = 0.0;//0.001; // please input the value <-------------------
  //    exp_config->output_directory = "/home/user/Documents/rosbag_python_evaluation_output/"; // please input the value
  //    <------------------- exp_config->experiment_name = "DA+ODO_A_noise_span_test"; // please input the value <-------------------
  // endregion

  // region*** PLOTTED setting for Experiment 2: # known SF vs SF ADD out vs. ADD noise (folded here)
  //
  //    // experiments control variables
  //    exp_config->start_noise_DA = 0.0000; // please input the value <-------------------
  //    exp_config->finish_noise_DA = 0.0000; // do not change
  //    exp_config->step_noise_DA = 0.0001; // do not change
  //    exp_config->finish_num_points = 500; // please input the value <------ TODO: CHECK always <= m2_pcd->points_.size()
  //    exp_config->start_num_points = exp_config->finish_num_points; // TODO: CHECK value != 0 & >= start_num_known_SF;
  //    exp_config->step_num_points = 1; // do not change
  //    exp_config->start_num_known_SF = 0; // please input the value <-------------------
  //    exp_config->finish_num_known_SF = 20; // please input the value <-------------------
  //    exp_config->step_num_known_SF = 1; // do not change
  //    exp_config->start_noise_SD = 0.0; // please input the value <-------------------
  //    exp_config->finish_noise_SD = 0.001; // do not change
  //    exp_config->step_noise_SD = 0.0001; // do not change
  //    exp_config->start_alpha = 0.0; // please input the value <-------------------
  //    exp_config->finish_alpha = exp_config->start_alpha; // do not change
  //    exp_config->step_alpha = 0.01; // do not change
  //    exp_config->start_seed = 0; // please input the value <-------------------
  //    exp_config->finish_seed = 9; // do not change
  //    exp_config->tot_time_rep = 0; // (n-1) please input the value <-------------------
  //    output.ADD_input_rounding = 1e-4; // please input the value <-------------------
  //    // cost blocks scales
  //    exp_config->alpha_pointsA = 1.0; // do not change
  //    exp_config->alpha_pointsB = 1.0; // do not change
  //    exp_config->alpha_DataAssC0 = 1.0; // do not change
  //    exp_config->alpha_DataAssC1 = 1.0; // do not change
  //    exp_config->alpha_chain = 0.0; // please input the value <-------------------
  //    exp_config->alpha_SF = 1.0; // please input the value <-------------------
  //    exp_config->alpha_odoA = 0.0; // please input the value <-------------------
  //    exp_config->alpha_odoB = 0.0; // please input the value <-------------------
  //    exp_config->alpha_C2C0 = 0.0; // please input the value <-------------------
  //    exp_config->alpha_C2C1 = 0.0; // please input the value <-------------------
  //    exp_config->output_directory = "/home/user/Documents/rosbag_python_evaluation_output/"; // please input the value
  //    <------------------- exp_config->experiment_name = "known_points_test"; // please input the value <-------------------
  // endregion

  // region*** setting for plot: x) # (alltogether vs. num points 5 known SF
  //
  //    // experiments control variables
  //    exp_config->start_noise_DA = 0.001; // please input the value <-------------------
  //    exp_config->finish_noise_DA = 0.001; // do not change
  //    exp_config->step_noise_DA = 0.01; // do not change
  //    exp_config->finish_num_points = 500; // please input the value <------ TODO: CHECK always <= m2_pcd->points_.size()
  //    exp_config->start_num_points = 500; // TODO: CHECK value != 0 & >= start_num_known_SF;
  //    exp_config->step_num_points = 1;
  //    exp_config->start_num_known_SF = 0; // please input the value <-------------------
  //    exp_config->finish_num_known_SF = 500; // please input the value <-------------------
  //    exp_config->step_num_known_SF = 1; // do not change
  //    exp_config->start_noise_SD = 0.001; // please input the value <-------------------
  //    exp_config->finish_noise_SD = 0.001; // do not change
  //    exp_config->step_noise_SD = 0.01; // do not change
  //    exp_config->start_alpha = 1.0; // please input the value <-------------------
  //    exp_config->finish_alpha = 1.0; // do not change
  //    exp_config->step_alpha = 0.5; // do not change
  //    exp_config->start_seed = 0; // please input the value <-------------------
  //    exp_config->finish_seed = 9; // do not change
  //    exp_config->tot_time_rep = 0; // (n-1) please input the value <-------------------
  ////    output.ADD_input_rounding = 1e-4;
  //    // cost blocks scales
  //    exp_config->alpha_pointsA = 1.0; // do not change
  //    exp_config->alpha_pointsB = 1.0; // do not change
  //    exp_config->alpha_DataAssC0 = 1.0; // block estimating x5
  //    exp_config->alpha_DataAssC1 = 1.0; // block estimating x6
  //    exp_config->alpha_SF = 1.0; // please input the value <-------------------
  //    exp_config->alpha_chain = 1.0; // please input the value <-------------------
  //    exp_config->alpha_odoA = 1.0; // please input the value <-------------------
  //    exp_config->alpha_odoB = 1.0; // please input the value <-------------------
  //    exp_config->alpha_C2C0 = 1.0; // please input the value <-------------------
  //    exp_config->alpha_C2C1 = 1.0;//0.001; // please input the value <-------------------
  //    exp_config->output_directory = "/home/user/Documents/rosbag_python_evaluation_output/"; // please input the value
  //    <------------------- exp_config->experiment_name = "alltogether0-500_+_points_span_test"; // please input the value
  //    <-------------------
  // endregion

  // region*** setting for plot: x) # (alltogether vs. num points 100 known SF
  //
  //    // experiments control variables
  //    exp_config->start_noise_DA = 0.001; // please input the value <-------------------
  //    exp_config->finish_noise_DA = 0.001; // do not change
  //    exp_config->step_noise_DA = 0.01; // do not change
  //    exp_config->finish_num_points = 500; // please input the value <------ TODO: CHECK always <= m2_pcd->points_.size()
  //    exp_config->start_num_points = 100; // TODO: CHECK value != 0 & >= start_num_known_SF;
  //    exp_config->step_num_points = 1;
  //    exp_config->start_num_known_SF = 100; // please input the value <-------------------
  //    exp_config->finish_num_known_SF = 100; // please input the value <-------------------
  //    exp_config->step_num_known_SF = 1; // do not change
  //    exp_config->start_noise_SD = 0.001; // please input the value <-------------------
  //    exp_config->finish_noise_SD = 0.001; // do not change
  //    exp_config->step_noise_SD = 0.01; // do not change
  //    exp_config->start_alpha = 1.0; // please input the value <-------------------
  //    exp_config->finish_alpha = 1.0; // do not change
  //    exp_config->step_alpha = 0.5; // do not change
  //    exp_config->start_seed = 10; // please input the value <-------------------
  //    exp_config->finish_seed = exp_config->start_seed; // do not change
  //    exp_config->tot_time_rep = 0; // (n-1) please input the value <-------------------
  //    // cost blocks scales
  //    exp_config->alpha_pointsA = 1.0; // do not change
  //    exp_config->alpha_pointsB = 1.0; // do not change
  //    exp_config->alpha_DataAssC0 = 1.0; // block estimating x5
  //    exp_config->alpha_DataAssC1 = 1.0; // block estimating x6
  //    exp_config->alpha_SF = 1.0; // please input the value <-------------------
  //    exp_config->alpha_chain = 0.0; // please input the value <-------------------
  //    exp_config->alpha_odoA = 0.005; // please input the value <-------------------
  //    exp_config->alpha_odoB = 0.0; // please input the value <-------------------
  //    exp_config->alpha_C2C0 = 0.0; // please input the value <-------------------
  //    exp_config->alpha_C2C1 = 0.0;//0.001; // please input the value <-------------------
  //    exp_config->output_directory = "/home/user/Documents/rosbag_python_evaluation_output/"; // please input the value
  //    <------------------- exp_config->experiment_name = "alltogether100_+_points_span_test"; // please input the value
  //    <-------------------
  // endregion

  // region*** setting for plot: x) # (alltogether vs. num points vs. time
  //
  //    // experiments control variables
  //    exp_config->start_noise_DA = 0.001; // please input the value <-------------------
  //    exp_config->finish_noise_DA = 0.001; // do not change
  //    exp_config->step_noise_DA = 0.01; // do not change
  //    exp_config->finish_num_points = 100000; // please input the value <------ TODO: CHECK always <= m2_pcd->points_.size()
  //    exp_config->start_num_points = 10; // TODO: CHECK value != 0 & >= start_num_known_SF;
  //    exp_config->step_num_points = 1000;
  //    exp_config->start_num_known_SF = 5; // please input the value <-------------------
  //    exp_config->finish_num_known_SF = 5; // please input the value <-------------------
  //    exp_config->step_num_known_SF = 1; // do not change
  //    exp_config->start_noise_SD = 0.001; // please input the value <-------------------
  //    exp_config->finish_noise_SD = 0.001; // do not change
  //    exp_config->step_noise_SD = 0.01; // do not change
  //    exp_config->start_alpha = 1.0; // please input the value <-------------------
  //    exp_config->finish_alpha = 1.0; // do not change
  //    exp_config->step_alpha = 0.5; // do not change
  //    exp_config->start_seed = 10; // please input the value <-------------------
  //    exp_config->finish_seed = exp_config->start_seed; // do not change
  //    exp_config->tot_time_rep = 0; // (n-1) please input the value <-------------------
  //    // cost blocks scales
  //    exp_config->alpha_pointsA = 1.0; // do not change
  //    exp_config->alpha_pointsB = 1.0; // do not change
  //    exp_config->alpha_DataAssC0 = 1.0; // block estimating x5
  //    exp_config->alpha_DataAssC1 = 1.0; // block estimating x6
  //    exp_config->alpha_SF = 1.0; // please input the value <-------------------
  //    exp_config->alpha_chain = 0.0; // please input the value <-------------------
  //    exp_config->alpha_odoA = 0.005; // please input the value <-------------------
  //    exp_config->alpha_odoB = 0.0; // please input the value <-------------------
  //    exp_config->alpha_C2C0 = 0.0; // please input the value <-------------------
  //    exp_config->alpha_C2C1 = 0.0;//0.001; // please input the value <-------------------
  //    exp_config->output_directory = "/home/user/Documents/rosbag_python_evaluation_output/"; // please input the value
  //    <------------------- exp_config->experiment_name = "alltogether5_+_points_vs_time_span_test"; // please input the value
  //    <-------------------
  // endregion

  // region*** post-MICCAI setting for various preliminary experiments

  // experiments control variables
  exp_config->start_noise_DA = 0.0;       // please input the value <-------------------
  exp_config->finish_noise_DA = 0.0;      // do not change
  exp_config->step_noise_DA = 0.00001;    // do not change
  exp_config->finish_num_points = 30000;  // please input the value <------ TODO: CHECK always <= m2_pcd->points_.size()
  exp_config->start_num_points = exp_config->finish_num_points;  // TODO: CHECK value != 0 & >= start_num_known_SF;
  exp_config->step_num_points = 1;                               // do not change
  exp_config->start_num_known_SF = 0;                            // please input the value <-------------------
  exp_config->finish_num_known_SF = 0;                           // please input the value <-------------------
  exp_config->step_num_known_SF = 1;                             // do not change
  exp_config->start_noise_SD = 0.0;                              // please input the value <-------------------
  exp_config->finish_noise_SD = 0.0;                             // do not change
  exp_config->step_noise_SD = 0.1;                               // do not change
  exp_config->start_alpha = 1.0;                                 // please input the value <-------------------
  exp_config->finish_alpha = 1.0;                                // do not change
  exp_config->step_alpha = 0.5;                                  // do not change
  exp_config->start_seed = 9;                                    // please input the value <-------------------
  exp_config->finish_seed = 9;                                   // do not change
  exp_config->tot_time_rep = 0;                                  // (n-1) please input the value <-------------------
  // cost blocks scales
  exp_config->alpha_pointsA = 0.0;    // do not change
  exp_config->alpha_pointsB = 0.0;    // do not change
                                      //    exp_config->alpha_points_labels = 1.0; // do not change
  exp_config->alpha_DataAssC0 = 0.0;  // block estimating x5
  exp_config->alpha_DataAssC1 = 0.0;  // block estimating x6
  exp_config->alpha_SF = 0.0;         // please input the value <-------------------
  exp_config->alpha_chain = 0.0;      // please input the value <-------------------
  exp_config->alpha_odoA = 0.0;       // please input the value <-------------------
  exp_config->alpha_odoB = 1.0;       // please input the value <-------------------
  exp_config->alpha_C2C0 = 0.0;       // please input the value <-------------------
  exp_config->alpha_C2C1 = 0.0;       // 0.001; // please input the value <-------------------
  exp_config->output_directory =
      "/home/";  // please input the value <-------------------
  //    exp_config->experiment_name = "relative_SF_denoise_test"; // please input the value <-------------------
  exp_config->experiment_name = "odo_and_labels_denoise_test";  // please input the value <-------------------

  // endregion
  // endregion
}

}  // namespace core
