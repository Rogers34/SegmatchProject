#ifndef SEGMATCH_SEGMENTER_HPP_
#define SEGMATCH_SEGMENTER_HPP_

#include <string>
#include <vector>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>


class Segmenter {
 public:
  /// \brief Segment the point cloud.
  virtual void segment(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, std::vector<pcl::PointIndices>& clusters) = 0;

}; // class Segmenter

#endif // SEGMATCH_SEGMENTER_HPP_
