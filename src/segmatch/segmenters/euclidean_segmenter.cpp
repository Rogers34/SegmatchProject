#include "segmatch/segmenters/euclidean_segmenter.hpp"

EuclideanSegmenter::EuclideanSegmenter()  {}

EuclideanSegmenter::EuclideanSegmenter(const SegMatchParam& param):
        kd_tree_(new pcl::search::KdTree<pcl::PointXYZI>){
	euclidean_cluster_extractor_.setClusterTolerance(param.ec_tolerance);
	euclidean_cluster_extractor_.setMinClusterSize(param.ec_min_cluster_size);
	euclidean_cluster_extractor_.setMaxClusterSize(param.ec_max_cluster_size);
	euclidean_cluster_extractor_.setSearchMethod(kd_tree_);
}

EuclideanSegmenter::~EuclideanSegmenter() 
{
	kd_tree_.reset();
}

void EuclideanSegmenter::segment(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, 
									std::vector<pcl::PointIndices>& clusters)
{
	clusters.clear();

	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
	*cloud_ptr = *cloud;

	euclidean_cluster_extractor_.setInputCloud(cloud_ptr);
	euclidean_cluster_extractor_.extract(clusters);
}

