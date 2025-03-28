

#include "multiviperfrog/core/workflow_manager.h"

namespace core {

WorkFlowManager::WorkFlowManager(ros::NodeHandlePtr nh) : nh_(nh),
      exp_config_(*nh) // <-- Pass NodeHandle here
{
  exp_config_.loadRosParams(*nh_);
}

// region***  global variables (folded here)
config::RosToPreprocessingData WorkFlowManager::Ros_input_global_buffer_;
// Initialization of static variables
int WorkFlowManager::camera_id_ = 0;  // TODO: make smarter for multiple cameras, for now just one is ok for only odometry
bool WorkFlowManager::callback_data_received_ = false;
bool WorkFlowManager::rosbag_data_available_ = false;
int WorkFlowManager::num_ros_data_received_ = 0;
int WorkFlowManager::ros_processing_window_size = 2;
int WorkFlowManager::ros_processing_window_step = 1;
bool WorkFlowManager::isFirstTimeRunning_ = true;
int WorkFlowManager::frame_index_counter_ = 0;
int WorkFlowManager::pair_index_counter_ = 0;
int WorkFlowManager::snippet_start_index_ = 0;
int WorkFlowManager::snippet_end_index_ = 0;
// Initialize empty vectors
std::vector<int> WorkFlowManager::snippet_start_indices_ = std::vector<int>();
std::vector<int> WorkFlowManager::snippet_end_indices_ = std::vector<int>();
// Estimates
std::vector<Eigen::Isometry3d> WorkFlowManager::cam1_snippet_start_poses_est_ = std::vector<Eigen::Isometry3d>();
std::vector<Eigen::Isometry3d> WorkFlowManager::cam1_snippet_end_poses_est_ = std::vector<Eigen::Isometry3d>();
std::vector<Eigen::Isometry3d> WorkFlowManager::cam2_snippet_start_poses_est_ = std::vector<Eigen::Isometry3d>();
std::vector<Eigen::Isometry3d> WorkFlowManager::cam2_snippet_end_poses_est_ = std::vector<Eigen::Isometry3d>();
// GT
std::vector<Eigen::Isometry3d> WorkFlowManager::cam1_snippet_start_poses_gt_ = std::vector<Eigen::Isometry3d>();
std::vector<Eigen::Isometry3d> WorkFlowManager::cam1_snippet_end_poses_gt_ = std::vector<Eigen::Isometry3d>();
std::vector<Eigen::Isometry3d> WorkFlowManager::cam2_snippet_start_poses_gt_ = std::vector<Eigen::Isometry3d>();
std::vector<Eigen::Isometry3d> WorkFlowManager::cam2_snippet_end_poses_gt_ = std::vector<Eigen::Isometry3d>();
// SVD
std::vector<Eigen::Isometry3d> WorkFlowManager::cam1_snippet_start_poses_svd_ = std::vector<Eigen::Isometry3d>();
std::vector<Eigen::Isometry3d> WorkFlowManager::cam1_snippet_end_poses_svd_ = std::vector<Eigen::Isometry3d>();
std::vector<Eigen::Isometry3d> WorkFlowManager::cam2_snippet_start_poses_svd_ = std::vector<Eigen::Isometry3d>();
std::vector<Eigen::Isometry3d> WorkFlowManager::cam2_snippet_end_poses_svd_ = std::vector<Eigen::Isometry3d>();

// Initialize time point
std::chrono::time_point<std::chrono::system_clock> WorkFlowManager::start_again_;
// Initialize input and output adapters
std::shared_ptr<interfaces::InputInterface<config::PreprocessingToOptimizationData>> offline_input;
std::shared_ptr<interfaces::InputInterface<config::RosToPreprocessingData>> ros_input;
std::shared_ptr<interfaces::InputInterface<config::HoloscanToPreprocessingData>> holoscan_input;
std::shared_ptr<interfaces::OutputInterface<config::PostprocessingToRosData, config::ExperimentConfig>> ros_output;
std::shared_ptr<interfaces::OutputInterface<config::PostprocessingToHoloscanData, config::ExperimentConfig>> holoscan_output;
// data structures
config::RosToPreprocessingData WorkFlowManager::ros_to_preprocessing_data_;
std::deque<config::RosToPreprocessingData> WorkFlowManager::ros_to_preprocessing_data_buffer_;
config::PreprocessingToOptimizationData WorkFlowManager::preprocessing_to_optimization_data_;
config::OptimizationToPostprocessingData WorkFlowManager::optimization_to_postprocessing_data_;
config::PostprocessingOdometryBuffer WorkFlowManager::postprocessing_odometry_buffer_;
config::PostprocessingData WorkFlowManager::postprocessing_data_;
config::PostprocessingToRosData WorkFlowManager::postprocessing_to_ros_data_;
config::CeresConfig WorkFlowManager::base_config_;

// endregion

void WorkFlowManager::updateRosInputFields(config::RosToPreprocessingData& data) {
  WorkFlowManager::Ros_input_global_buffer_ = data;
  WorkFlowManager::callback_data_received_ = true;
}

void WorkFlowManager::runMultiViPerFrOG() {
  // region***  manual setting of high level options (folded here)
  bool use_offline_input = false;    // implemented but not tested, probably buggy
  bool use_ros_input = true;         // implemented
  bool use_ros_output = true;        // implemented
  bool experiment_mode = false;      // not implemented yet
  bool use_holoscan_input = false;   // not implemented yet
  bool use_holoscan_output = false;  // not implemented yet

  // TODO: check bug with Rosbag processor, the optimization outputs frame_id of cam 2, while if "false" outputs cam 1
  const bool isProcessAsFastAsPossible = true;  // if true, reads from rosbag, otherwise from camera(s)
  const double topics_sync_upper_thr = 0.0;     // max delay in seconds for the topics to be synchronized
  ros_processing_window_size = 2;               // how many messages are accumulated before sending to preprocessing
  ros_processing_window_step = 1;               // how many message are deleted from the buffer after sending to preprocessing
  // endregion
  //--------------------------------------------------------------------------------------------------------------
  // region*** initialization of variables that live across processing core iterations (folded here)
  //  initialize odometry buffer //TODO: find a more elegant way to do this initialization!
  postprocessing_odometry_buffer_ = config::PostprocessingOdometryBuffer();
  for (int i = 0; i < config::max_num_cameras; i++) {
    postprocessing_odometry_buffer_.Cpose_start__T__Ccurrent_estim[i].matrix().setIdentity();
  }
  // do some sanity checks on the settings
  if (ros_processing_window_step > ros_processing_window_size) {
    throw std::runtime_error("WorkFlowManager: ros_processing_window_step cannot be greater than ros_processing_window_size.");
  } else if (ros_processing_window_step == 0) {
    throw std::runtime_error("WorkFlowManager: ros_processing_window_step cannot be zero.");
  }
  if (camera_id_ >= exp_config_.num_cameras) {
    throw std::runtime_error("WorkFlowManager: camera_id_ cannot be greater than exp_config_.num_cameras.");
  }
  // TODO: note, now implemented just to have both poses (blender output),
  //  later for real sensor data this buffer can be used to store other inputs
  // endregion
  //--------------------------------------------------------------------------------------------------------------
  // region***  processing logics that call the processing core (folded here)

  if (use_offline_input) {
    offline_input = std::make_shared<interfaces::OfflineInputAdapter>();
    if (!offline_input->initialize()) {
      throw std::runtime_error("WorkFlowManager: failed to initialize Offline input adapter.");
    }
  }
  if (use_ros_input) {
    ros_input = std::make_shared<interfaces::ROSInputAdapter>(nh_, this, isProcessAsFastAsPossible, topics_sync_upper_thr);
    std::cout << "WorkFlowManager: initializing ROSInputAdapter..." << std::endl;
    // Downcast to ROSInputAdapter to access derived methods
    auto downcasted_ros_input = std::dynamic_pointer_cast<interfaces::ROSInputAdapter>(ros_input);

    if (downcasted_ros_input && !ros_input->initialize()) {
      throw std::runtime_error("WorkFlowManager: failed to initialize ROS input adapter.");
    } else if (downcasted_ros_input) {
      sharedDataProcessor = downcasted_ros_input->getDataProcessor();
      //                downcasted_ros_input.reset(); // Release the downcasted pointer TODO: dangerous if accessing this in the future
      std::cout << "WorkFlowManager: done!" << std::endl;
    } else {
      throw std::runtime_error("WorkFlowManager: failed to downcast ROS input adapter.");
    }
  }
  if (use_holoscan_input) {
    holoscan_input = std::make_shared<interfaces::HoloscanInputAdapter>();
    if (!holoscan_input->initialize()) {
      throw std::runtime_error("WorkFlowManager: failed to initialize Holoscan input adapter.");
    }
  }
  if (use_ros_output) {
    // if ros is used also for input, then the data processor is already initialized and therefore passed)
    if (use_ros_input) {
      ros_output = std::make_shared<interfaces::ROSOutputAdapter>(nh_, sharedDataProcessor);
    } else {
      ros_output = std::make_shared<interfaces::ROSOutputAdapter>(nh_);
    }
    if (!ros_output->initialize()) {
      throw std::runtime_error("WorkFlowManager: failed to initialize ROS output adapter.");
    }
  }
  if (use_holoscan_output) {
    holoscan_output = std::make_shared<interfaces::HoloscanOutputAdapter>();
    if (!holoscan_output->initialize()) {
      throw std::runtime_error("WorkFlowManager: failed to initialize Holoscan output adapter.");
    }
  }

  // realtime and rosbag loops implemented here below
  if (use_offline_input) {
    // TODO: most probably still BUGGHY, check and make so that it reproduces the full MICCAI result.
    std::cout << "WorkFlowManager: running in offline mode..." << std::endl;
    config::PreprocessingToOptimizationData file_to_optimization_data;
    while (ros::ok()) {
      file_to_optimization_data = config::PreprocessingToOptimizationData();  // Reset data

      if (offline_input && !offline_input->getData(file_to_optimization_data)) {
        std::cerr << "WorkFlowManager: data not available from offline input. Exiting..." << std::endl;
        break;
      }
      std::cerr << "WorkFlowManager: data available from offline input. Running optimization..." << std::endl;
      // Run optimization
      core::runExperimentMICCAI(base_config_, exp_config_, file_to_optimization_data);

      std::cerr << "Optimization done." << std::endl;

      // TODO implement some output strategy
      //                if (ros_output && !ros_output->sendData(data)) {
      //                    std::cerr << "Failed to send data through ROS output adapter." << std::endl;
      //                }
    }
  } else if (use_ros_input) {
    if (isProcessAsFastAsPossible) {
      std::cout << "WorkFlowManager: waiting for rosbag messages..." << std::endl;
      // ROS bag processing
      if (!ros_input->getData(ros_to_preprocessing_data_)) {
        // workflow explained:
        // this will call RosRosbagDataProcessor::startProcessing which will call readRosbagWithSync
        // which will call core::WorkFlowManager::runProcessingCore() until the rosbag is finished
        std::cout << "WorkFlowManager: all done, bye bye." << std::endl;
      } else {
        throw std::runtime_error("WorkFlowManager: BUG -- should not have rosbag data here yet.");
      }
    } else {
      std::cout << "WorkFlowManager: waiting for ros real-time messages..." << std::endl;
      // ROS realtime processing
      while (ros::ok()) {
        ros::spinOnce();
        runProcessingCore();
        // workflow explained:
        // this will call ros_input->getData which will check if the callback_data_received_ flag is true and return the data
        // callback_data_received_ is set to true in core::WorkFlowManager::updateRosInputFields
        // which is called by RosOnlineDataProcessor::syncCallback every time the callback is called
      }
      throw std::runtime_error("WorkFlowManager: user terminated manually, exiting...");
    }

  } else {
    // TODO: here not implemented Holoscan input
    throw std::runtime_error("No input adapter selected.");
  }
  // endregion
}

void WorkFlowManager::runProcessingCore() {
  // Printout of index counter
  std::cout << "WorkFlowManager: frame index counter: " << frame_index_counter_ << std::endl;

  // region***  data loading and preprocessing (folded here)
  bool data_available = true;
  ros_to_preprocessing_data_ = config::RosToPreprocessingData();                    // Reset data
  preprocessing_to_optimization_data_ = config::PreprocessingToOptimizationData();  // Reset data
  // function simply returns when data is not available
  if (ros_input && !ros_input->getData(ros_to_preprocessing_data_)) {
    data_available = false;
  }
  if (!data_available) {
    return;
  }
  // function proceeds when data is available
  auto start = std::chrono::high_resolution_clock::now();
  // update preprocessing buffer
  num_ros_data_received_++;
  if (num_ros_data_received_ <= ros_processing_window_size) {
    //            std::cout << "WorkFlowManager: received data #" << num_ros_data_received_ << " from ROSInputAdapter."
    //                      << std::endl;
    ros_to_preprocessing_data_buffer_.push_back(ros_to_preprocessing_data_);
    if (num_ros_data_received_ == ros_processing_window_size) {
      //                std::cout << "WorkFlowManager: preprocessing..." << std::endl;
      // preprocess data
      // TODO: parallelize with OpenMP the call to preprocessRosDataBuffer for each camera (if multiple cameras)
      for (int i = 0; i < exp_config_.num_cameras; i++) {
        core::preprocessing::preprocessRosDataBuffer(ros_to_preprocessing_data_buffer_, exp_config_, &preprocessing_to_optimization_data_,
                                                     i);
        //                    std::cout << "WorkFlowManager: preprocessing done for camera #" << i << std::endl;
      }
      // reset the callback counter
      num_ros_data_received_ = ros_processing_window_size - ros_processing_window_step;
      // remove the oldest data from the buffer
      while (ros_to_preprocessing_data_buffer_.size() > num_ros_data_received_) {
        ros_to_preprocessing_data_buffer_.pop_front();
      }
    } else {  // Main exit in first iteration
      frame_index_counter_++;
      return;
    }
  } else {  // throw error, should not happen
    throw std::runtime_error("Received more data than expected, some bug is around.");
  }

  // Very first iteration --> also start of first snippet
  if (isFirstTimeRunning_) {
    std::cout << "\033[1;32m-----------------------------------------------------------------------------------------------" << std::endl;
    std::cout << "WorkFlowManager: received enough scans. Actual processing starting at pair index counter: " << pair_index_counter_
              << std::endl;
    std::cout << "-----------------------------------------------------------------------------------------------\033[0m" << std::endl;
  }

  // Snippet is starting
  bool snippet_is_starting = (frame_index_counter_ % exp_config_.snippet_length == 0);
  if (snippet_is_starting) {
    std::cout << "\033[1;32m-----------------------------------------------------------------------------------------------" << std::endl;
    std::cout << "WorkFlowManager: snippet is starting. Actual processing starting at pair index counter: " << pair_index_counter_
              << std::endl;
    std::cout << "-----------------------------------------------------------------------------------------------\033[0m" << std::endl;
    snippet_start_index_ = frame_index_counter_;
  }

  // Snippet is over
  bool snippet_is_over = (frame_index_counter_ == snippet_start_index_ + (exp_config_.snippet_length - 1));
  if (snippet_is_over) {
    std::cout << "\033[1;32m-----------------------------------------------------------------------------------------------" << std::endl;
    std::cout << "WorkFlowManager: snippet is over. Actual processing ending at pair index counter: " << pair_index_counter_ << std::endl;
    std::cout << "-----------------------------------------------------------------------------------------------\033[0m" << std::endl;
    snippet_end_index_ = frame_index_counter_;
    // Log Start and End of Snippet
    snippet_start_indices_.push_back(snippet_start_index_);
    snippet_end_indices_.push_back(snippet_end_index_);
  }

  // Down-sample Point Clouds
  if (exp_config_.downsample_before_preprocessing) {
    throw std::runtime_error("Preprocessing: downsampling_before_preprocessing not with optimize_multicam, fix logic");
  } else if (!exp_config_.downsample_after_preprocessing) {
    throw std::runtime_error("WorkFlowManager: downsampling strategy not selected, proceed with full clouds???");
  }
  //TODO: check integrity of tool masks and confidence values, then integrate them in the downsampling process!!
  // print how many zeros in the tool mask
  std::cout << "WorkFlowManager: tool mask zeros: " << std::count(preprocessing_to_optimization_data_.tool_mask[0]->begin() + 1,
                                                               preprocessing_to_optimization_data_.tool_mask[0]->end(), 0)
            << std::endl;
//  // throw debug error
//  throw(std::runtime_error("WorkFlowManager: debug stop."));

  for (int i = 0; i < exp_config_.num_cameras; i++) {
    //                    std::cout << "WorkFlowManager: downsampling pointcloud for camera #" << i << std::endl;
    core::preprocessing::downsamplePointClouds(preprocessing_to_optimization_data_, exp_config_, i);
  }
  if (exp_config_.optimize_multicam) {
    if (exp_config_.num_cameras != 2) {  // TODO not tested for cases with more than 2 cameras, hardcoded
      throw std::runtime_error("WorkFlowManager: number of cameras >2 not supported yet.");
    }
    int cam_id_self = 0;  // TODO make global and managed by some logic!
    int cam_id_other = 1;
    //                        std::cout << "WorkFlowManager: computing GT camera to camera transformation..." << std::endl;
    // compute GT camera to camera transformations at t0 and t1 (see note inside the function for future)
    core::preprocessing::computeCam2Cam(&preprocessing_to_optimization_data_,
                                        exp_config_, cam_id_self, cam_id_other, false, true); //TODO: testing and fixing ICP
    // compute overlap points between the two cameras // here exposing the struct content,
    // TODO wrap in a function if >2 cameras to get all overlap permutations, things get MESSY!
    core::preprocessing::getOverlapIndices(this, preprocessing_to_optimization_data_, exp_config_, cam_id_self, cam_id_other);
    //  this is the function that stores exp_config_.num_points_overlap[]
    core::preprocessing::extractOverlapPoints(preprocessing_to_optimization_data_, exp_config_, cam_id_self,
                                              cam_id_other);  // TODO make more efficient with select_by_index

    if (exp_config_.optimize_odometries) {
      // in this case we select in the overlap given the points of the downsampled clouds used in odo_opt
      std::cout << "WorkFlowManager: index sampling overlap region for camera # 0 and 1" << std::endl;
      core::preprocessing::indexSamplingOverlapRegion(this, preprocessing_to_optimization_data_, exp_config_, cam_id_self, cam_id_other);
    } else {  // if no odometry optimization, then just randomly downsample the overlap
              // extract and save the overlap points
      if (exp_config_.num_points_overlap[0] < 10) {
        std::cout << "WorkFlowManager: " << exp_config_.num_points_overlap[0] << " points in overlap" << std::endl;
        throw std::runtime_error("WorkFlowManager: <10 points in overlap, check the data/settings.");
      } else {
        //                            std::cout << "WorkFlowManager: " << exp_config_.num_points_overlap[0]
        //                                      << " points in overlap" << std::endl;
      }
      //      std::cout << "WorkFlowManager: random downsampling overlap region for camera # 0 and 1" << std::endl;
      //      core::preprocessing::downsampleOverlapRegion(preprocessing_to_optimization_data_, exp_config_, cam_id_self, cam_id_other);
    }
  } else {
    //                    std::cout << "WorkFlowManager: multicam optimization not selected, skipping c2c and overlap..."
    //                              << std::endl;
  }
  if (!exp_config_.optimize_odometries && !exp_config_.optimize_multicam && !exp_config_.no_optimization_just_processing) {
    throw std::runtime_error("WorkFlowManager: downsampling & opt options not selected correctly, check exp_config.");
  }

  // End timer
  auto end_preprocess = std::chrono::high_resolution_clock::now();
  // Calculate elapsed time
  std::chrono::duration<double> elapsed_preprocess = (end_preprocess - start);  // seconds
  //        std::cout << "WorkFlowManager: Execution time (runProcessingCore:preprocessing:wholeblock total): " <<
  //        elapsed_preprocess.count() * 1000 << " milliseconds" << std::endl;
  // endregion
  //--------------------------------------------------------------------------------------------------------------
  // region***  plot with o3d the downsampled pointclouds (folded here)
  if (exp_config_.optimize_multicam) {
    //            // plot with o3d the downsampled pointclouds
    //            // set colors of preprocessing_to_optimization_data_.Cn0__t__Cn0_P0_o3d_overlap_ptr[0] to red
    //            preprocessing_to_optimization_data_.Cn0__t__Cn0_P0_o3d_overlap_ptr[0]->colors_.assign(
    //                    preprocessing_to_optimization_data_.Cn0__t__Cn0_P0_o3d_overlap_ptr[0]->points_.size(),
    //                    Eigen::Vector3d(1.0, 0.0, 0.0)
    //            );
    //            preprocessing_to_optimization_data_.Cn0__t__Cn0_P0_o3d_overlap_downsampled_ptr[0]->colors_.assign(
    //                    preprocessing_to_optimization_data_.Cn0__t__Cn0_P0_o3d_overlap_downsampled_ptr[0]->points_.size(),
    //                    Eigen::Vector3d(0.0, 0.0, 1.0)
    //            );
    //            open3d::visualization::DrawGeometries(
    //                    {preprocessing_to_optimization_data_.Cn0__t__Cn0_P0_o3d_downsampled_ptr[0],
    //                     preprocessing_to_optimization_data_.Cn0__t__Cn0_P0_o3d_overlap_ptr[0],
    //                     preprocessing_to_optimization_data_.Cn0__t__Cn0_P0_o3d_overlap_downsampled_ptr[0]});

    // set colors of preprocessing_to_optimization_data_.Cn0__t__Cn0_P0_o3d_overlap_ptr[1] to red
    //            preprocessing_to_optimization_data_.Cn0__t__Cn0_P0_o3d_overlap_ptr[1]->colors_.assign(
    //                    preprocessing_to_optimization_data_.Cn0__t__Cn0_P0_o3d_overlap_ptr[1]->points_.size(),
    //                    Eigen::Vector3d(1.0, 0.0, 0.0)
    //            );
    //            preprocessing_to_optimization_data_.Cn0__t__Cn0_P0_o3d_overlap_downsampled_ptr[1]->colors_.assign(
    //                    preprocessing_to_optimization_data_.Cn0__t__Cn0_P0_o3d_overlap_downsampled_ptr[1]->points_.size(),
    //                    Eigen::Vector3d(0.0, 1.0, 0.0)
    //            );
    //            open3d::visualization::DrawGeometries(
    //                    {preprocessing_to_optimization_data_.Cn0__t__Cn0_P0_o3d_downsampled_ptr[1],
    //                     preprocessing_to_optimization_data_.Cn0__t__Cn0_P0_o3d_overlap_ptr[1],
    //                     preprocessing_to_optimization_data_.Cn0__t__Cn0_P0_o3d_overlap_downsampled_ptr[1]});
    //
    //            // create a new pointcloud of same size
    //            auto overlap_downsampled_ptr = std::make_shared<open3d::geometry::PointCloud>();
    //            // deepcopy  preprocessing_to_optimization_data_.Cn0__t__Cn0_P0_o3d_overlap_downsampled_ptr[1] to overlap_downsampled_ptr
    //            *overlap_downsampled_ptr = *preprocessing_to_optimization_data_.Cn0__t__Cn0_P0_o3d_overlap_downsampled_ptr[1];
    //            // transform the pointcloud to the right
    //            overlap_downsampled_ptr->Transform(preprocessing_to_optimization_data_.Cself0__T__Cother0[0].matrix());
    //            // plot both time 0 and 1
    //            open3d::visualization::DrawGeometries(
    //                    {preprocessing_to_optimization_data_.Cn0__t__Cn0_P0_o3d_overlap_downsampled_ptr[0],
    //                     overlap_downsampled_ptr});
    //
  }
  // endregion
  //--------------------------------------------------------------------------------------------------------------
  // region***  Ceres optimization (folded here)
  optimization_to_postprocessing_data_ = config::OptimizationToPostprocessingData();  // Reset data
  if (exp_config_.optimize_odometries) {
    // prepare experiment config
    // right now hardcoded in the original struct, then make dynamic again with above function for specific usages
    //            core::setupExperiment(&base_config_, &exp_config_); //TODO call this setupExperimentOdometries, if any custom setup needed
    //            std::cout << "WorkFlowManager: running first step of optimization (odometries)." << std::endl;
    // Run odometry optimization
    for (int i = 0; i < exp_config_.num_cameras; i++) {
      camera_id_ = i;
      // initialize optimization parameters (this is currently hardcoded in runExperimentSingleOdometry) TODO: make smarter
      core::runExperimentSingleOdometry(base_config_, exp_config_, preprocessing_to_optimization_data_,
                                        optimization_to_postprocessing_data_, camera_id_);
    }  // TODO add saving of odometry output as init of next step (check for conflict with core::initializeOptimizationParameters BELOW)
  }
  if (exp_config_.optimize_multicam) {
    if (exp_config_.num_cameras > 1 && exp_config_.num_cameras < 3) {  // TODO not tested for cases with more than 2 cameras, hardcoded
      int cam_id_self = 0;
      int cam_id_other = 1;
      // prepare experiment config
      // core::setupExperiment(&base_config_, &exp_config_); //TODO call this setupExperimentMulticam, if any custom setup needed
      //                std::cout << "WorkFlowManager: running second step of optimization (multicam)." << std::endl;
      // Run multicam optimization
      core::runExperimentTwoCameras(base_config_, exp_config_, preprocessing_to_optimization_data_, optimization_to_postprocessing_data_,
                                    cam_id_self, cam_id_other);
    } else {
      throw std::runtime_error("WorkFlowManager: number of cameras <2 (no multicam) or >2 (not supported yet).");
    }
  } else {
    //            std::cout << "WorkFlowManager: optimization not selected, proceeding with simple processing..." << std::endl;
  }
  //        std::cout << "WorkFlowManager: optimization done." << std::endl;
  // endregion
  //--------------------------------------------------------------------------------------------------------------
  // region***  postprocessing and data output (folded here)
  auto start_postprocess = std::chrono::high_resolution_clock::now();
  postprocessing_to_ros_data_ = config::PostprocessingToRosData();  // Reset data
  postprocessing_data_ = config::PostprocessingData();              // Reset data

  for (int i = 0; i < exp_config_.num_cameras; i++) {
    camera_id_ = i;
    // postprocessing
    //        std::cout << "WorkFlowManager: running postprocessing..." << std::endl;

    // GT odometry (uncomment, WORKING)
    //  hardcoded odometry value to GT tf value for testing (bypassing the optimization)
    optimization_to_postprocessing_data_.Ccurrent__T__Cnext_GT[camera_id_].matrix() =
        preprocessing_to_optimization_data_.Cn0__T__Cn1[camera_id_].matrix();
    optimization_to_postprocessing_data_.Ccurrent__T__Cnext_GT[camera_id_].stamp_ =
        preprocessing_to_optimization_data_.world__T__Cn1[camera_id_].stamp_;
    optimization_to_postprocessing_data_.Ccurrent__T__Cnext_GT[camera_id_].frame_id_ =
        preprocessing_to_optimization_data_.world__T__Cn1[camera_id_].frame_id_ + "_GT";

    if (exp_config_.run_odo_SVD) {
      // SVD/Horn (w/wo ransac) odometry (uncomment, WORKING)
      // NOTE: RANSAC has very poor implementation, tod0 swap with https://github.com/tsattler/RansacLib
      core::preprocessing::computeOdometryRigidClosedForm(preprocessing_to_optimization_data_, exp_config_,
                                                          &optimization_to_postprocessing_data_, camera_id_, true, false);
    }
    if (exp_config_.run_odo_ICP) {
      // ICP odometry (uncomment, WORKING)
      core::preprocessing::computeOdometryRigidIterative(preprocessing_to_optimization_data_, exp_config_,
                                                         &optimization_to_postprocessing_data_,  // TODO: change to passing a reference
                                                         camera_id_);
    }

    // Accumulate odometry
    // hardcoded exception: for the first frame, the odometry is the world__T__Ca0 GT pose
    if (isFirstTimeRunning_) {
      //            std::cout << "WorkFlowManager: first frame, setting odometry to GT value." << std::endl;
      core::postprocessing::initializeOdometry(preprocessing_to_optimization_data_,
                                               &postprocessing_odometry_buffer_,  // TODO: change to passing a reference
                                               camera_id_);
    }

    // set manually the parent frame id
    postprocessing_odometry_buffer_.parent_frame_id[camera_id_] = "world";

    // save current odometry for output
    // TODO NOTE IMPORTANT, since the algorithm is using forward SF, the odometry (and therefore also the pointcloud stamps)
    //  need to be published at the time of "previous", so first publish odometry, then update it!
    core::postprocessing::saveOdometry(postprocessing_odometry_buffer_,
                                       &postprocessing_data_,  // TODO: change to passing a reference
                                       camera_id_);

    // save cloud to output for visualization
    // NOTE the two clouds are the same if downsample_before_preprocessing = true;
    postprocessing_data_.Cn0__t__Cn0_P0_o3d_ptr = preprocessing_to_optimization_data_.Cn0__t__Cn0_P0_o3d_ptr;
    postprocessing_data_.Cn0__t__Cn0_P0_o3d_downsampled_ptr = preprocessing_to_optimization_data_.Cn0__t__Cn0_P0_o3d_downsampled_ptr;
//    postprocessing_data_.Cn0__t__Cn0_P0_o3d_downsampled_ptr[camera_id_]->colors_[0] = Eigen::Vector3d(0.0, 0.0, 1.0);  // blue
//    // print value of label of the first point of the downsampled cloud
//    std::cout << "WorkFlowManager: label of the first point of the downsampled cloud: "
//              << optimization_to_postprocessing_data_.rigidity_labels_n_estim[camera_id_][0] << std::endl;
    postprocessing_data_.Cn0__t__Cn0_P0_o3d_overlap_ptr = preprocessing_to_optimization_data_.Cn0__t__Cn0_P0_o3d_overlap_ptr;
    postprocessing_data_.Cn0__t__Cn0_P0_o3d_overlap_downsampled_ptr =
        preprocessing_to_optimization_data_.Cn0__t__Cn0_P0_o3d_overlap_downsampled_ptr;
    //        /////// debug plotting of the GT sceneflow
    //        // create a pointcloud2 as hardcopy of the points of Cn0__t__Cn0_P0_o3d_ptr
    //        auto o3d_pc = std::make_shared<open3d::geometry::PointCloud>();
    //        *o3d_pc = *postprocessing_data_.Cn0__t__Cn0_P0_o3d_downsampled_ptr[camera_id_];  // Deep copy
    //        for (int i = 0; i < o3d_pc->points_.size(); i++) {
    //            o3d_pc->points_[i] =  o3d_pc->points_[i] + o3d_pc->normals_[i];
    //        }
    //        utils::PlotCloudsCorrespondences(o3d_pc, postprocessing_data_.Cn0__t__Cn0_P0_o3d_downsampled_ptr[camera_id_]);
    //        ///////

    // convert odometry to ROS message (we publish the odometry as accumulate in the buffer in the previous iteration)
    core::postprocessing::postprocessOptimizationDataForRos(postprocessing_data_, postprocessing_to_ros_data_, exp_config_, camera_id_);
    core::postprocessing::saveEigenIsometryStampedToFreiburgStyle(postprocessing_data_, camera_id_, frame_index_counter_, exp_config_);

    // see note above, with the Blender data there will be a delay of 1 frame + optimization time w.r.t. the realtime stream
    // in the real stream (not blender) there will be 1 frame + scene flow estimation time + optimization time.
    // here instead we receive the frame 0 with already the scene flow, but have to wait for frame 2 anyway for the GT camera pose.
    // the odometry is estimated but published in the next iteration as pose at time 0 (previous).
    // update odometry with the new estimated transformation
    core::postprocessing::updateOdometry(optimization_to_postprocessing_data_,
                                         &postprocessing_odometry_buffer_,  // TODO: change to passing a reference
                                         camera_id_);

    //        std::cout << "WorkFlowManager: postprocessing done." << std::endl;
    //        std::cout << "WorkFlowManager: sending data to ROSOutputAdapter..." << std::endl;
  }  // end of the loop over cameras

  // Output results to ROS
  if (ros_output &&
      !ros_output->sendData(postprocessing_to_ros_data_, exp_config_)) {  // TODO make this dynamic with camera_id, now badly hardcoded
    throw std::runtime_error("Failed to send data through ROS output adapter.");
  }

  // End timer
  auto end_total = std::chrono::high_resolution_clock::now();
  // Calculate elapsed time
  std::chrono::duration<double> elapsed_postprocess = (end_total - start_postprocess);  // seconds
  std::chrono::duration<double> elapsed_total = (end_total - start);                    // seconds
  if (!isFirstTimeRunning_) {
    std::chrono::duration<double> elapsed_total_n_loop = (start - start_again_);  // seconds
    //            std::cout << "WorkFlowManager: Execution time (runProcessingCore: 1/Hz - total): " << elapsed_total_n_loop.count() * 1000
    //            << " milliseconds" << std::endl;
  } else {
    isFirstTimeRunning_ = false;  // set to false after the first run to avoid reinitializing the odometry to GT
    snippet_is_starting = true;
  }
  // 02-2025 TODO NOTE: this part of the code below is actually missing the pose of the first frame!!!

  //        std::cout << "WorkFlowManager: Execution time (runProcessingCore:postprocessing): " << elapsed_postprocess.count() * 1000 << "
  //        milliseconds" << std::endl;
  std::cout << "WorkFlowManager: Execution time (runProcessingCore:total): " << elapsed_total.count() * 1000 << " milliseconds"
            << std::endl;
  start_again_ = std::chrono::high_resolution_clock::now();
  // endregion
  //--------------------------------------------------------------------------------------------------------------
  // region***  logging and bookkeeping (folded here)
  // Print Counter
  std::cout << "Pair Index Counter: " << pair_index_counter_ << std::endl;

  // Counter increment
  ++frame_index_counter_;
  ++pair_index_counter_;

  // Show length of odometry buffer
  std::cout << "Odometry Buffer Length: " << postprocessing_odometry_buffer_.Cpose_start__T__Ccurrent_estim.size() << std::endl;
  std::cout << "Odometry GT Buffer Length: " << postprocessing_odometry_buffer_.Cpose_start__T__Ccurrent_GT.size() << std::endl;

  if (snippet_is_starting) {
    // Estimated
    cam1_snippet_start_poses_est_.push_back(postprocessing_odometry_buffer_.Cpose_start__T__Ccurrent_estim[0]);
    cam2_snippet_start_poses_est_.push_back(postprocessing_odometry_buffer_.Cpose_start__T__Ccurrent_estim[1]);
    // GT
    cam1_snippet_start_poses_gt_.push_back(postprocessing_odometry_buffer_.Cpose_start__T__Ccurrent_GT[0]);
    cam2_snippet_start_poses_gt_.push_back(postprocessing_odometry_buffer_.Cpose_start__T__Ccurrent_GT[1]);
    // SVD
    cam1_snippet_start_poses_svd_.push_back(postprocessing_odometry_buffer_.Cpose_start__T__Ccurrent_svd[0]);
    cam2_snippet_start_poses_svd_.push_back(postprocessing_odometry_buffer_.Cpose_start__T__Ccurrent_svd[1]);
  }
  if (snippet_is_over) {
    // Estimated
    cam1_snippet_end_poses_est_.push_back(postprocessing_odometry_buffer_.Cpose_start__T__Ccurrent_estim[0]);
    cam2_snippet_end_poses_est_.push_back(postprocessing_odometry_buffer_.Cpose_start__T__Ccurrent_estim[1]);
    // GT
    cam1_snippet_end_poses_gt_.push_back(postprocessing_odometry_buffer_.Cpose_start__T__Ccurrent_GT[0]);
    cam2_snippet_end_poses_gt_.push_back(postprocessing_odometry_buffer_.Cpose_start__T__Ccurrent_GT[1]);
    // SVD
    cam1_snippet_end_poses_svd_.push_back(postprocessing_odometry_buffer_.Cpose_start__T__Ccurrent_svd[0]);
    cam2_snippet_end_poses_svd_.push_back(postprocessing_odometry_buffer_.Cpose_start__T__Ccurrent_svd[1]);

    // reset position of cameras after eachsnipped in the published Tf. (the metrics above are already computed in this way)
    if (exp_config_.show_reset_pose_between_snippets) {
      for (int i = 0; i < exp_config_.num_cameras; i++) {
        core::postprocessing::initializeOdometry(preprocessing_to_optimization_data_,
                                                 &postprocessing_odometry_buffer_,  // TODO: change to passing a reference
                                                 i);
      }
    }
  }
  // endregion
}

}  // namespace core
