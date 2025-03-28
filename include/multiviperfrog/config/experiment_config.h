

#ifndef CONFIG_EXPERIMENT_CONFIG_H
#define CONFIG_EXPERIMENT_CONFIG_H

// System
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <vector>

// ROS
#include <ros/package.h>
#include <ros/ros.h>
namespace config {

// region*** Odometry and/or multicamera online optimization, no experiment (folded here)
struct ExperimentConfig {
 public:
  // Constructor
  ExperimentConfig(const ros::NodeHandle& nh) {
    // Package Path
    package_path = ros::package::getPath("multiviperfrog");
    // Full path to the YAML configuration file
    std::string yamlFilePath = package_path + "/config/config.yaml";
    std::cout << "RosDataProcessor: yaml file path: " << yamlFilePath << std::endl;
    // Load the YAML configuration file
    yamlConfig_ = YAML::LoadFile(yamlFilePath);

    // Take the values from the YAML file
    // Setup
    num_cameras = yamlConfig_["num_cameras"].as<int>();
    std::cout << "num_cameras: " << num_cameras << std::endl;
    snippet_length = yamlConfig_["snippet_length"].as<int>();
    std::cout << "snippet_length: " << snippet_length << std::endl;
    show_reset_pose_between_snippets = yamlConfig_["show_reset_pose_between_snippets"].as<bool>();
    std::cout << "show_reset_pose_between_snippets: " << show_reset_pose_between_snippets << std::endl;

    // Optimization parameters
    optimize_odometries = yamlConfig_["optimize_odometries"].as<bool>();
    std::cout << "optimize_odometries: " << optimize_odometries << std::endl;
    optimize_multicam = yamlConfig_["optimize_multicam"].as<bool>();
    std::cout << "optimize_multicam: " << optimize_multicam << std::endl;
    no_optimization_just_processing = yamlConfig_["no_optimization_just_processing"].as<bool>();
    std::cout << "no_optimization_just_processing: " << no_optimization_just_processing << std::endl;
    run_odo_SVD = yamlConfig_["run_odo_SVD"].as<bool>();
    std::cout << "run_odo_SVD: " << run_odo_SVD << std::endl;
    run_odo_ICP = yamlConfig_["run_odo_ICP"].as<bool>();
    std::cout << "run_odo_ICP: " << run_odo_ICP << std::endl;
    // Robust optimization parameters
    use_huber_loss = yamlConfig_["use_huber_loss"].as<bool>();
    std::cout << "use_huber_loss: " << use_huber_loss << std::endl;
    huber_loss_threshold = yamlConfig_["huber_loss_threshold"].as<double>();
    std::cout << "huber_loss_threshold: " << huber_loss_threshold << std::endl;
    // Outlier rejection parameters
    lambda_deformation = yamlConfig_["lambda_deformation"].as<double>();
    std::cout << "lambda_deformation: " << lambda_deformation << std::endl;
    lambda_regularization = yamlConfig_["lambda_regularization"].as<double>();
    std::cout << "lambda_regularization: " << lambda_regularization << std::endl;
    max_abs_SF = yamlConfig_["max_abs_SF"].as<double>();
    std::cout << "max_abs_SF: " << max_abs_SF << std::endl;
    rigidity_label_update_interval = yamlConfig_["rigidity_label_update_interval"].as<int>();
    std::cout << "rigidity_label_update_interval: " << rigidity_label_update_interval << std::endl;
    steepness_rigidity_label_trust = yamlConfig_["steepness_rigidity_label_trust"].as<double>();
    std::cout << "steepness_rigidity_label_trust: " << steepness_rigidity_label_trust << std::endl;
    ratio_rigidity_label_trust_a = yamlConfig_["ratio_rigidity_label_trust_a"].as<double>();
    std::cout << "ratio_rigidity_label_trust_a: " << ratio_rigidity_label_trust_a << std::endl;
    ratio_rigidity_label_trust_b = yamlConfig_["ratio_rigidity_label_trust_b"].as<double>();
    std::cout << "ratio_rigidity_label_trust_b: " << ratio_rigidity_label_trust_b << std::endl;
    use_absflow_GT_from_render_as_knowledge = yamlConfig_["use_absflow_GT_from_render_as_knowledge"].as<bool>();
    std::cout << "use_absflow_GT_from_render_as_knowledge: " << use_absflow_GT_from_render_as_knowledge << std::endl;
    use_absflow_estimate_from_odometry_as_knowledge = yamlConfig_["use_absflow_estimate_from_odometry_as_knowledge"].as<bool>();
    std::cout << "use_absflow_estimate_from_odometry_as_knowledge: " << use_absflow_estimate_from_odometry_as_knowledge << std::endl;
    set_to_zero_flow_points_with_high_label_as_knowledge = yamlConfig_["set_to_zero_flow_points_with_high_label_as_knowledge"].as<bool>();
    std::cout << "set_to_zero_flow_points_with_high_label_as_knowledge: " << set_to_zero_flow_points_with_high_label_as_knowledge
              << std::endl;
    steepness_percentile_labels = yamlConfig_["steepness_percentile_labels"].as<double>();
    std::cout << "steepness_percentile_labels: " << steepness_percentile_labels << std::endl;
    ratio_percentile_labels_a = yamlConfig_["ratio_percentile_labels_a"].as<double>();
    std::cout << "ratio_percentile_labels_a: " << ratio_percentile_labels_a << std::endl;
    ratio_percentile_labels_b = yamlConfig_["ratio_percentile_labels_b"].as<double>();
    std::cout << "ratio_percentile_labels_b: " << ratio_percentile_labels_b << std::endl;
    labels_upper_percentile = yamlConfig_["labels_upper_percentile"].as<double>();
    std::cout << "labels_upper_percentile: " << labels_upper_percentile << std::endl;
    ratio_upper_threshold_labels_a = yamlConfig_["ratio_upper_threshold_labels_a"].as<double>();
    std::cout << "ratio_upper_threshold_labels_a: " << ratio_upper_threshold_labels_a << std::endl;
    ratio_upper_threshold_labels_b = yamlConfig_["ratio_upper_threshold_labels_b"].as<double>();
    std::cout << "ratio_upper_threshold_labels_b: " << ratio_upper_threshold_labels_b << std::endl;
    labels_higher_threshold = yamlConfig_["labels_higher_threshold"].as<double>();
    std::cout << "labels_higher_threshold: " << labels_higher_threshold << std::endl;

    // Point Cloud Parameters
    cloud_original_height = yamlConfig_["cloud_original_height"].as<int>();
    std::cout << "cloud_original_height: " << cloud_original_height << std::endl;
    cloud_original_width = yamlConfig_["cloud_original_width"].as<int>();
    std::cout << "cloud_original_width: " << cloud_original_width << std::endl;
    points_overlap_threshold = yamlConfig_["points_overlap_threshold"].as<double>();
    std::cout << "points_overlap_threshold: " << points_overlap_threshold << std::endl;
    downsampledcloud_to_overlap_threshold = yamlConfig_["downsampledcloud_to_overlap_threshold"].as<double>();
    std::cout << "downsampledcloud_to_overlap_threshold: " << downsampledcloud_to_overlap_threshold << std::endl;
    seed_random_downsampling = yamlConfig_["seed_random_downsampling"].as<int>();
    std::cout << "seed_random_downsampling: " << seed_random_downsampling << std::endl;
    desired_downsampled_points_number_uniform = yamlConfig_["desired_downsampled_points_number_uniform"].as<int>();
    std::cout << "desired_downsampled_points_number_uniform: " << desired_downsampled_points_number_uniform << std::endl;
    desired_downsampled_points_number_random = yamlConfig_["desired_downsampled_points_number_random"].as<int>();
    std::cout << "desired_downsampled_points_number_random: " << desired_downsampled_points_number_random << std::endl;
    downsample_before_preprocessing = yamlConfig_["downsample_before_preprocessing"].as<bool>();
    std::cout << "downsample_before_preprocessing: " << downsample_before_preprocessing << std::endl;
    downsample_after_preprocessing = yamlConfig_["downsample_after_preprocessing"].as<bool>();
    std::cout << "downsample_after_preprocessing: " << downsample_after_preprocessing << std::endl;
    downsample_ICP = yamlConfig_["downsample_ICP"].as<bool>();
    std::cout << "downsample_ICP: " << downsample_ICP << std::endl;

    // Cost Blocks Scales
    alpha_RCM = yamlConfig_["alpha_RCM"].as<double>();
    std::cout << "alpha_RCM: " << alpha_RCM << std::endl;
    alpha_pointsA = yamlConfig_["alpha_pointsA"].as<double>();
    std::cout << "alpha_pointsA: " << alpha_pointsA << std::endl;
    alpha_pointsB = yamlConfig_["alpha_pointsB"].as<double>();
    std::cout << "alpha_pointsB: " << alpha_pointsB << std::endl;
    alpha_DataAssC0 = yamlConfig_["alpha_DataAssC0"].as<double>();
    std::cout << "alpha_DataAssC0: " << alpha_DataAssC0 << std::endl;
    alpha_DataAssC1 = yamlConfig_["alpha_DataAssC1"].as<double>();
    std::cout << "alpha_DataAssC1: " << alpha_DataAssC1 << std::endl;
    alpha_SF_A = yamlConfig_["alpha_SF_A"].as<double>();
    std::cout << "alpha_SF_A: " << alpha_SF_A << std::endl;
    alpha_SF_B = yamlConfig_["alpha_SF_B"].as<double>();
    std::cout << "alpha_SF_B: " << alpha_SF_B << std::endl;
    alpha_SF = yamlConfig_["alpha_SF"].as<double>();
    std::cout << "alpha_SF: " << alpha_SF << std::endl;
    alpha_chain = yamlConfig_["alpha_chain"].as<double>();
    std::cout << "alpha_chain: " << alpha_chain << std::endl;
    alpha_odoA = yamlConfig_["alpha_odoA"].as<double>();
    std::cout << "alpha_odoA: " << alpha_odoA << std::endl;
    alpha_odoB = yamlConfig_["alpha_odoB"].as<double>();
    std::cout << "alpha_odoB: " << alpha_odoB << std::endl;
    alpha_C2C0 = yamlConfig_["alpha_C2C0"].as<double>();
    std::cout << "alpha_C2C0: " << alpha_C2C0 << std::endl;
    alpha_C2C1 = yamlConfig_["alpha_C2C1"].as<double>();
    std::cout << "alpha_C2C1: " << alpha_C2C1 << std::endl;
  }

  void loadRosParams(const ros::NodeHandle& nh) { //TODO move other values here
    nh.getParam("optimize_multicam", optimize_multicam);
    std::cout << "optimize_multicam (after ROS override): " << optimize_multicam << std::endl;
  }

  // Setup
  int num_cameras = 2;
  int snippet_length = 5;
  bool show_reset_pose_between_snippets = false;
  std::string package_path;

  // Optimization
  bool optimize_odometries = true;  // TODO test output with false and all combinations
  bool optimize_multicam = true;
  bool no_optimization_just_processing = true;
  bool run_odo_SVD = false;
  bool run_odo_ICP = false;
  // Robust Loss
  bool use_huber_loss = false;
  double huber_loss_threshold = 0.0001;
  // Outlier Rejection
  double lambda_deformation = 1.0;          // if = 0, all labels will tend to 1 (fully rigid)
  double lambda_regularization = 0.000001;  // if = 0, all labels will tend to 0 (fully deforming)//0.0001, worked with odoBsupervision
                                            // (avg. label 0.33); 0.0005 worked without odoB supervision (avg. label 0.69)
  double max_abs_SF = 0.001;                  // max of absolute tissue deformation between two frames
  int rigidity_label_update_interval =
      1;  // 1 for every frame, 2 for every other frame (one frame has SetParameterBlockConstant(rigidity_label.data())
  double steepness_rigidity_label_trust = 1.0;
  double ratio_rigidity_label_trust_a = 1.0;  // alpha_odo = rigidity_label_mean * rigidity_label_trust_ratio
  double ratio_rigidity_label_trust_b = 1.0;  // alpha_odo = rigidity_label_mean * rigidity_label_trust_ratio
  bool use_absflow_GT_from_render_as_knowledge = true;
  bool use_absflow_estimate_from_odometry_as_knowledge = false;
  bool set_to_zero_flow_points_with_high_label_as_knowledge = false;
  double steepness_percentile_labels = 1.0;
  double ratio_percentile_labels_a = 1.0;
  double ratio_percentile_labels_b = 1.0;
  double labels_upper_percentile = 90.0;  // used in case the previous is true
  double ratio_upper_threshold_labels_a = 1.0;
  double ratio_upper_threshold_labels_b = 1.0;
  double labels_higher_threshold = 0.9;

  // Point Cloud Parameters
  int cloud_original_height = 360;
  int cloud_original_width = 640;
  double points_overlap_threshold = 0.00005;
  double downsampledcloud_to_overlap_threshold = 0.0005;
  int seed_random_downsampling = 99;                     // same as in the MICCAI experiments
  int desired_downsampled_points_number_uniform = 1000;  // used for full point cloud of each camera
  int desired_downsampled_points_number_random = 1000;   // used for overlap region
  // use the next two in alternative! eventually both false if just performing downsample for ICP only
  bool downsample_before_preprocessing = false;
  bool downsample_after_preprocessing = true;
  bool downsample_ICP = false;

  // cost blocks scales
  double alpha_RCM = 0.0;
  double alpha_pointsA = 1.0;
  double alpha_pointsB = 1.0;
  double alpha_DataAssC0 = 1.0;
  double alpha_DataAssC1 = 1.0;
  double alpha_SF_A = 0.0;  // defined at runtime
  double alpha_SF_B = 0.0;  // defined at runtime
  double alpha_SF = 0.0;  // left for compatibility with MICCAI experiments, and with Problems cost blocks (in multicam defined at runtime)
  double alpha_chain = 1.0;
  double alpha_odoA = 0.0;  // defined at runtime
  double alpha_odoB = 0.0;  // defined at runtime
  double alpha_C2C0 = 0.0;
  double alpha_C2C1 = 0.0;
  // runtime defined variables
  std::vector<int> num_points_overlap{
      3};  // theoretically 3 overlap regions with 3 cameras, could also be 2. for now just using element [0]
  int num_points_overlap_downsampled;
  int num_points_fullcloud_downsampled;
  int num_points;  // IMPORTANT!! this is used by the Problems!!! needs to be set with either of the above somwhere in the wrkflow manager
                   // or the optimization function itself.!
  // region*** not used for now, but needed for compilation (folded here)
  //  TODO fix by creating some sort of struct that initializes all the variables, and overwrite somewhere else
  //  only if using the native O3D downsampling function, does not allow for points index retriveal
  std::string downsample_strategy;  // "uniform", "random", "voxel" (last two only for ICP)
  double downsample_interval;       // 220 produces ~ 1000 points
  double downsample_ratio;          // 0.01 only for ICP with "random"
  double downsample_voxel_size;     // 0.003 only for ICP with "voxel"
  // experiments control variables
  int start_num_points;
  int finish_num_points;
  int step_num_points;
  int start_num_known_SF;
  int num_known_SF;  // iterable
  int finish_num_known_SF;
  int step_num_known_SF;
  double start_noise_SD;
  double noise_SD;  // iterable
  double finish_noise_SD;
  double step_noise_SD;
  double start_alpha;
  double alpha_span;  // iterable
  double finish_alpha;
  double step_alpha;
  int max_points_selection_number;  // TODO: NOTE make sure its < number of knn found pairs w. max span noise!
  double start_noise_DA;
  double noise_DA_SD;  // iterable
  double finish_noise_DA;
  double step_noise_DA;
  int start_seed;
  int seed;  // iterable
  int finish_seed;
  int time_rep;  // iterable
  int tot_time_rep;
  std::string output_directory;
  std::string experiment_name;
  bool header_flag;

 private:
  // Yaml config
  YAML::Node yamlConfig_;
};
// endregion

// region*** Post-MICCAI settings (folded here)
//    struct ExperimentConfig { // post-MICCAI settings
//        int num_cameras = 2;
//        int cloud_height = 360;
//        int cloud_width = 640;
//        int desired_downsampled_points_number = 1000;
//        bool downsample_before_preprocessing = false;
//        bool downsample_after_preprocessing = false;
//        bool downsample_ICP = true;
//        std::string downsample_strategy = "uniform"; // "uniform", "random", "voxel" (last two only for ICP)
//        double downsample_interval = 1;
//        double downsample_ratio = 0.01; // only for ICP with "random"
//        double downsample_voxel_size = 0.003; // only for ICP with "voxel"
//        // cost blocks scales
//        double alpha_pointsA = 1.0;
//        double alpha_pointsB = 1.0;
////        double alpha_points_labels = 1.0;
//        double alpha_DataAssC0 = 1.0;
//        double alpha_DataAssC1 = 1.0;
//        double alpha_SF = 1.0;
//        double alpha_chain = 1.0;
//        double alpha_odoA = 0.0;
//        double alpha_odoB = 0.0;
//        double alpha_C2C0 = 0.0;
//        double alpha_C2C1 = 1.0;
//        // experiments control variables
//        int start_num_points;
//        int num_points = 0; // iterable
//        int finish_num_points;
//        int step_num_points;
//        int start_num_known_SF;
//        int num_known_SF = 0; // iterable
//        int finish_num_known_SF;
//        int step_num_known_SF;
//        double start_noise_SD;
//        double noise_SD = 0.0; // iterable
//        double finish_noise_SD;
//        double step_noise_SD;
//        double start_alpha;
//        double alpha_span = 0.0; // iterable
//        double finish_alpha;
//        double step_alpha;
//        double points_overlap_threshold = 0.00005;
//        int max_points_selection_number = 30000; //TODO: NOTE make sure its < number of knn found pairs w. max span noise!
//        double start_noise_DA;
//        double noise_DA_SD = 0.0; // iterable
//        double finish_noise_DA;
//        double step_noise_DA;
//        int start_seed;
//        int seed = 0 ; // iterable
//        int finish_seed;
//        int time_rep = 0; // iterable
//        int tot_time_rep;
//        std::string output_directory;
//        std::string experiment_name;
//        bool header_flag = false;
//    };
// endregion

struct output_settings {
  double ADD_input_rounding = 0.0;
  double ADD_metric_input_DA = 0.0;
  double total_avg_opt_time = 0.0;
  double ADD_metric_input_x1 = 0.0;
  double ADD_metric_output_x1 = 0.0;
  double ADD_metric_input_x2 = 0.0;
  double ADD_metric_output_x2 = 0.0;
  double ADD_metric_output_x2_ICP = 0.0;  // for o3d ICP
  double ADD_metric_input_x5 = 0.0;
  double ADD_metric_output_x5 = 0.0;
  double ADD_metric_input_x6 = 0.0;
  double ADD_metric_output_x6 = 0.0;
  double ADD_metric_input_SF = 0.0;
  double ADD_metric_output_SF = 0.0;

  double ADD_metric_input_DA_avg = 0.0;
  double ADD_metric_input_x1_avg = 0.0;
  double ADD_metric_output_x1_avg = 0.0;
  double ADD_metric_input_x2_avg = 0.0;
  double ADD_metric_output_x2_avg = 0.0;
  double ADD_metric_input_x5_avg = 0.0;
  double ADD_metric_output_x5_avg = 0.0;
  double ADD_metric_input_x6_avg = 0.0;
  double ADD_metric_output_x6_avg = 0.0;
  double ADD_metric_input_SF_avg = 0.0;
  double ADD_metric_output_SF_avg = 0.0;
};

}  // namespace config

#endif  // CONFIG_EXPERIMENT_CONFIG_H
