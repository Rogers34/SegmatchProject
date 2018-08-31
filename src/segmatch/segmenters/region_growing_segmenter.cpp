#include "segmatch/segmenters/region_growing_segmenter.hpp"


RegionGrowingSegmenter::RegionGrowingSegmenter()  {}

RegionGrowingSegmenter::RegionGrowingSegmenter(const SegMatchParam& param) :
        kd_tree_(new pcl::search::KdTree<pcl::PointXYZI>)
{

	if (param.rg_knn_for_normals == 0) 
	{
		normal_estimator_omp_.setRadiusSearch(param.rg_radius_for_normals);
		std::cout << "Normals estimation based on radius.";
	} 
	else 	
	{
		normal_estimator_omp_.setKSearch(param.rg_knn_for_normals);
		std::cout << "Normals estimation based on knn.";
	}
	normal_estimator_omp_.setSearchMethod(kd_tree_);
	// Ensure that the normals point to the same direction.
	normal_estimator_omp_.setViewPoint(std::numeric_limits<float>::max(),
										std::numeric_limits<float>::max(),
										std::numeric_limits<float>::max());

	region_growing_estimator_.setMinClusterSize(param.rg_min_cluster_size);
	region_growing_estimator_.setMaxClusterSize(param.rg_max_cluster_size);
	region_growing_estimator_.setSearchMethod(kd_tree_);
	region_growing_estimator_.setNumberOfNeighbours(param.rg_knn_for_growing);
	region_growing_estimator_.setSmoothnessThreshold(
		param.rg_smoothness_threshold_deg / 180.0 * 3.141592653);
	region_growing_estimator_.setCurvatureThreshold(param.rg_curvature_threshold);

	region_growing_estimator_.setSmoothModeFlag(true);
	region_growing_estimator_.setCurvatureTestFlag(true);
	region_growing_estimator_.setResidualTestFlag(false);

	m_rg_curvature_threshold = param.rg_curvature_threshold;
}

RegionGrowingSegmenter::~RegionGrowingSegmenter() {
  kd_tree_.reset();
}

void RegionGrowingSegmenter::segment(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, 
										std::vector<pcl::PointIndices>& clusters)
{
	// Clear segments.
	clusters.clear();

	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
	*cloud_ptr = *cloud;

	pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
	normal_estimator_omp_.setInputCloud(cloud_ptr);
	normal_estimator_omp_.compute(*normals);


	// Remove points with high curvature.
	pcl::IndicesPtr indices(new std::vector <int>);
	for (size_t i = 0u; i < normals->size(); ++i) 
	{
		if (normals->points[i].curvature < m_rg_curvature_threshold) 
		{
			indices->push_back(i);
		}
	}

	region_growing_estimator_.setInputCloud(cloud_ptr);
	region_growing_estimator_.setInputNormals(normals);
	region_growing_estimator_.setIndices(indices);
	region_growing_estimator_.extract(clusters);
}


