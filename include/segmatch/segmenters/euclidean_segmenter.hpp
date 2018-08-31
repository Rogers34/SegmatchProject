#ifndef SEGMATCH_EUCLIDEAN_SEGMENTER_HPP_
#define SEGMATCH_EUCLIDEAN_SEGMENTER_HPP_

#include <string>

#include <pcl/point_types.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include "segmatch/SegMatchParam.hpp"
#include "segmatch/segmenters/segmenter.hpp"


class EuclideanSegmenter : public Segmenter 
{
public:
	EuclideanSegmenter();
	explicit EuclideanSegmenter(const SegMatchParam& param);
	~EuclideanSegmenter();

	/// \brief Segment the point cloud.
	virtual void segment(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, 
							std::vector<pcl::PointIndices>& clusters);
private:

	// PCL object members.
	pcl::search::KdTree<pcl::PointXYZI>::Ptr kd_tree_;
	pcl::EuclideanClusterExtraction<pcl::PointXYZI> euclidean_cluster_extractor_;
}; // class EuclideanSegmenter


#endif // SEGMATCH_EUCLIDEAN_SEGMENTER_HPP_
