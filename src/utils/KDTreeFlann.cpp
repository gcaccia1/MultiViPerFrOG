

// #include "multiviperfrog/utils/KDTreeFlann.h"
// namespace utils {
//
//     KDTreeFlann::KDTreeFlann() {}
//
//     KDTreeFlann::KDTreeFlann(const Eigen::MatrixXd &data) { SetMatrixData(data); }
//
//     KDTreeFlann::KDTreeFlann(const open3d::geometry::Geometry &geometry) {
//         SetGeometry(geometry);
//     }
//
//     KDTreeFlann::KDTreeFlann(const open3d::pipelines::registration::Feature &feature) {
//         SetFeature(feature);
//     }
//
//     KDTreeFlann::~KDTreeFlann() {}
//
//     bool KDTreeFlann::SetMatrixData(const Eigen::MatrixXd &data) {
//         return SetRawData(Eigen::Map<const Eigen::MatrixXd>(data.data(), data.rows(), data.cols()));
//     }
//
//     bool KDTreeFlann::SetGeometry(const open3d::geometry::Geometry &geometry) {
//         switch (geometry.GetGeometryType()) {
//             case open3d::geometry::Geometry::GeometryType::PointCloud:
//                 return SetRawData(Eigen::Map<const Eigen::MatrixXd>(
//                         (const double *)((const open3d::geometry::PointCloud &)geometry).points_.data(),
//                         3, ((const open3d::geometry::PointCloud &)geometry).points_.size()));
//             case open3d::geometry::Geometry::GeometryType::TriangleMesh:
//             case open3d::geometry::Geometry::GeometryType::HalfEdgeTriangleMesh:
//                 return SetRawData(Eigen::Map<const Eigen::MatrixXd>(
//                         (const double *)((const open3d::geometry::TriangleMesh &)geometry).vertices_.data(),
//                         3, ((const open3d::geometry::TriangleMesh &)geometry).vertices_.size()));
//             case open3d::geometry::Geometry::GeometryType::Image:
//             case open3d::geometry::Geometry::GeometryType::Unspecified:
//             default:
//                 std::cerr << "[KDTreeFlann::SetGeometry] Unsupported Geometry type." << std::endl;
//                 return false;
//         }
//     }
//
//     bool KDTreeFlann::SetFeature(const open3d::pipelines::registration::Feature &feature) {
//         return SetMatrixData(feature.data_);
//     }
//
//     bool KDTreeFlann::SetRawData(const Eigen::Map<const Eigen::MatrixXd> &data) {
//         dimension_ = data.rows();
//         dataset_size_ = data.cols();
//         if (dimension_ == 0 || dataset_size_ == 0) {
//             std::cerr << "[KDTreeFlann::SetRawData] Failed due to no data." << std::endl;
//             return false;
//         }
//         data_.resize(dataset_size_ * dimension_);
//         memcpy(data_.data(), data.data(), dataset_size_ * dimension_ * sizeof(double));
//         data_interface_.reset(new Eigen::Map<const Eigen::MatrixXd>(data));
//         nanoflann_index_.reset(new KDTree_t(dimension_, std::cref(*data_interface_), 15));
//         nanoflann_index_->index_->buildIndex();
//         return true;
//     }
//
//     // If you need explicit template instantiations, add them below
//     template int KDTreeFlann::Search<Eigen::Vector3d>(
//             const Eigen::Vector3d &query,
//             const open3d::geometry::KDTreeSearchParam &param,
//             std::vector<int> &indices,
//             std::vector<double> &distance2) const;
//
//     template int KDTreeFlann::SearchKNN<Eigen::Vector3d>(
//             const Eigen::Vector3d &query,
//             int knn,
//             std::vector<int> &indices,
//             std::vector<double> &distance2) const;
//
//     template int KDTreeFlann::SearchRadius<Eigen::Vector3d>(
//             const Eigen::Vector3d &query,
//             double radius,
//             std::vector<int> &indices,
//             std::vector<double> &distance2) const;
//
//     template int KDTreeFlann::SearchHybrid<Eigen::Vector3d>(
//             const Eigen::Vector3d &query,
//             double radius,
//             int max_nn,
//             std::vector<int> &indices,
//             std::vector<double> &distance2) const;
//
// }  // namespace utils

#include "multiviperfrog/utils/KDTreeFlann.h"
#include <iostream>

namespace utils {

// Default Constructor
KDTreeFlann::KDTreeFlann() {}

// Parameterized Constructor for Matrix Data
KDTreeFlann::KDTreeFlann(const Eigen::MatrixXd& data) {
  SetMatrixData(data);
}

// Parameterized Constructor for Geometry
KDTreeFlann::KDTreeFlann(const open3d::geometry::Geometry& geometry) {
  SetGeometry(geometry);
}

// Parameterized Constructor for Feature
KDTreeFlann::KDTreeFlann(const open3d::pipelines::registration::Feature& feature) {
  SetFeature(feature);
}

// Destructor
KDTreeFlann::~KDTreeFlann() {}

// Sets the data for the KDTree from a matrix
bool KDTreeFlann::SetMatrixData(const Eigen::MatrixXd& data) {
  return SetRawData(Eigen::Map<const Eigen::MatrixXd>(data.data(), data.rows(), data.cols()));
}

// Sets the data for the KDTree from geometry
bool KDTreeFlann::SetGeometry(const open3d::geometry::Geometry& geometry) {
  switch (geometry.GetGeometryType()) {
    case open3d::geometry::Geometry::GeometryType::PointCloud:
      return SetRawData(Eigen::Map<const Eigen::MatrixXd>((const double*)((const open3d::geometry::PointCloud&)geometry).points_.data(), 3,
                                                          ((const open3d::geometry::PointCloud&)geometry).points_.size()));
    case open3d::geometry::Geometry::GeometryType::TriangleMesh:
    case open3d::geometry::Geometry::GeometryType::HalfEdgeTriangleMesh:
      return SetRawData(Eigen::Map<const Eigen::MatrixXd>((const double*)((const open3d::geometry::TriangleMesh&)geometry).vertices_.data(),
                                                          3, ((const open3d::geometry::TriangleMesh&)geometry).vertices_.size()));
    case open3d::geometry::Geometry::GeometryType::Image:
    case open3d::geometry::Geometry::GeometryType::Unspecified:
    default:
      std::cerr << "[KDTreeFlann::SetGeometry] Unsupported Geometry type." << std::endl;
      return false;
  }
}

// Sets the data for the KDTree from the feature data
bool KDTreeFlann::SetFeature(const open3d::pipelines::registration::Feature& feature) {
  return SetMatrixData(feature.data_);
}

// Internal method to set KDTree data
bool KDTreeFlann::SetRawData(const Eigen::Map<const Eigen::MatrixXd>& data) {
  dimension_ = data.rows();
  dataset_size_ = data.cols();
  if (dimension_ == 0 || dataset_size_ == 0) {
    std::cerr << "[KDTreeFlann::SetRawData] Failed due to no data." << std::endl;
    return false;
  }
  data_.resize(dataset_size_ * dimension_);
  memcpy(data_.data(), data.data(), dataset_size_ * dimension_ * sizeof(double));
  data_interface_.reset(new Eigen::Map<const Eigen::MatrixXd>(data));
  nanoflann_index_.reset(new KDTree_t(dimension_, std::cref(*data_interface_), 15));
  nanoflann_index_->index_->buildIndex();
  return true;
}

}  // namespace utils
