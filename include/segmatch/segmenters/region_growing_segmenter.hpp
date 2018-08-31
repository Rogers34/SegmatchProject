#ifndef SEGMATCH_REGION_GROWING_SEGMENTER_HPP_
#define SEGMATCH_REGION_GROWING_SEGMENTER_HPP_

#include <string>

#include <pcl/point_types.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/segmentation/region_growing.h>

#include "segmatch/SegMatchParam.hpp"
#include "segmatch/segmenters/segmenter.hpp"

class RegionGrowingSegmenter : public Segmenter 
{
public:
	RegionGrowingSegmenter();
	explicit RegionGrowingSegmenter(const SegMatchParam& param);
	~RegionGrowingSegmenter();

	/// \brief Segment the point cloud.
	virtual void segment(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, 
							std::vector<pcl::PointIndices>& clusters);

private:
	float m_rg_curvature_threshold;
	// PCL object members.
	pcl::search::KdTree<pcl::PointXYZI>::Ptr kd_tree_;
	pcl::NormalEstimationOMP<pcl::PointXYZI, pcl::Normal> normal_estimator_omp_;
	pcl::RegionGrowing<pcl::PointXYZI, pcl::Normal> region_growing_estimator_;
}; // class RegionGrowingSegmenter


#endif // SEGMATCH_REGION_GROWING_SEGMENTER_HPP_
