

// This custom KDTreeFlann.h is a modified version of the original header from Open3D.
// It relocates the implementation of the KDTreeFlann class and its templates into a custom namespace (utils)
// and provides explicit template instantiations for types such as Eigen::Block to avoid linking issues.

#ifndef UTILS_KDTREEFLANN_H
#define UTILS_KDTREEFLANN_H

#pragma once

#include <open3d/Open3D.h>
#include <Eigen/Core>
#include <iostream>
#include <memory>
#include <nanoflann.hpp>
#include <vector>

namespace nanoflann {
struct metric_L2;
template <class MatrixType, int DIM, class Distance, bool row_major>
struct KDTreeEigenMatrixAdaptor;
}  // namespace nanoflann

namespace utils {

class KDTreeFlann {
 public:
  /// Default Constructor.
  KDTreeFlann();
  /// Parameterized Constructor.
  KDTreeFlann(const Eigen::MatrixXd& data);
  /// Parameterized Constructor.
  KDTreeFlann(const open3d::geometry::Geometry& geometry);
  /// Parameterized Constructor.
  KDTreeFlann(const open3d::pipelines::registration::Feature& feature);
  ~KDTreeFlann();
  KDTreeFlann(const KDTreeFlann&) = delete;
  KDTreeFlann& operator=(const KDTreeFlann&) = delete;

  /// Sets the data for the KDTree from a matrix.
  bool SetMatrixData(const Eigen::MatrixXd& data);
  /// Sets the data for the KDTree from geometry.
  bool SetGeometry(const open3d::geometry::Geometry& geometry);
  /// Sets the data for the KDTree from the feature data.
  bool SetFeature(const open3d::pipelines::registration::Feature& feature);

  template <typename T>
  inline int Search(const T& query, const open3d::geometry::KDTreeSearchParam& param, std::vector<int>& indices,
                    std::vector<double>& distance2) const;

  template <typename T>
  inline int SearchKNN(const T& query, int knn, std::vector<int>& indices, std::vector<double>& distance2) const;

  template <typename T>
  inline int SearchRadius(const T& query, double radius, std::vector<int>& indices, std::vector<double>& distance2) const;

  template <typename T>
  inline int SearchHybrid(const T& query, double radius, int max_nn, std::vector<int>& indices, std::vector<double>& distance2) const;

 private:
  bool SetRawData(const Eigen::Map<const Eigen::MatrixXd>& data);

 protected:
  using KDTree_t = nanoflann::KDTreeEigenMatrixAdaptor<Eigen::Map<const Eigen::MatrixXd>, -1, nanoflann::metric_L2, false>;

  std::vector<double> data_;
  std::unique_ptr<Eigen::Map<const Eigen::MatrixXd>> data_interface_;
  std::unique_ptr<KDTree_t> nanoflann_index_;
  size_t dimension_ = 0;
  size_t dataset_size_ = 0;
};

// Inline template implementations in the header file

template <typename T>
inline int KDTreeFlann::Search(const T& query, const open3d::geometry::KDTreeSearchParam& param, std::vector<int>& indices,
                               std::vector<double>& distance2) const {
  switch (param.GetSearchType()) {
    case open3d::geometry::KDTreeSearchParam::SearchType::Knn:
      return SearchKNN(query, static_cast<const open3d::geometry::KDTreeSearchParamKNN&>(param).knn_, indices, distance2);
    case open3d::geometry::KDTreeSearchParam::SearchType::Radius:
      return SearchRadius(query, static_cast<const open3d::geometry::KDTreeSearchParamRadius&>(param).radius_, indices, distance2);
    case open3d::geometry::KDTreeSearchParam::SearchType::Hybrid:
      return SearchHybrid(query, static_cast<const open3d::geometry::KDTreeSearchParamHybrid&>(param).radius_,
                          static_cast<const open3d::geometry::KDTreeSearchParamHybrid&>(param).max_nn_, indices, distance2);
    default:
      return -1;
  }
}

template <typename T>
inline int KDTreeFlann::SearchKNN(const T& query, int knn, std::vector<int>& indices, std::vector<double>& distance2) const {
  if (data_.empty() || dataset_size_ <= 0 || size_t(query.rows()) != dimension_ || knn < 0) {
    return -1;
  }
  indices.resize(knn);
  distance2.resize(knn);
  std::vector<Eigen::Index> indices_eigen(knn);
  int k = nanoflann_index_->index_->knnSearch(query.data(), knn, indices_eigen.data(), distance2.data());
  indices.resize(k);
  distance2.resize(k);
  std::copy_n(indices_eigen.begin(), k, indices.begin());
  return k;
}

template <typename T>
inline int KDTreeFlann::SearchRadius(const T& query, double radius, std::vector<int>& indices, std::vector<double>& distance2) const {
  if (data_.empty() || dataset_size_ <= 0 || size_t(query.rows()) != dimension_) {
    return -1;
  }
  std::vector<nanoflann::ResultItem<Eigen::Index, double>> indices_dists;
  int k = nanoflann_index_->index_->radiusSearch(query.data(), radius * radius, indices_dists, nanoflann::SearchParameters(0.0));
  indices.resize(k);
  distance2.resize(k);
  for (int i = 0; i < k; ++i) {
    indices[i] = indices_dists[i].first;
    distance2[i] = indices_dists[i].second;
  }
  return k;
}

template <typename T>
inline int KDTreeFlann::SearchHybrid(const T& query, double radius, int max_nn, std::vector<int>& indices,
                                     std::vector<double>& distance2) const {
  if (data_.empty() || dataset_size_ <= 0 || size_t(query.rows()) != dimension_ || max_nn < 0) {
    return -1;
  }
  distance2.resize(max_nn);
  std::vector<Eigen::Index> indices_eigen(max_nn);
  int k = nanoflann_index_->index_->knnSearch(query.data(), max_nn, indices_eigen.data(), distance2.data());
  k = std::distance(distance2.begin(), std::lower_bound(distance2.begin(), distance2.begin() + k, radius * radius));
  indices.resize(k);
  distance2.resize(k);
  std::copy_n(indices_eigen.begin(), k, indices.begin());
  return k;
}

}  // namespace utils

#endif  // UTILS_KDTREEFLANN_H
