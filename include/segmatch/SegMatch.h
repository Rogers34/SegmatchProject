#pragma once
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/filters/extract_indices.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/visualization/cloud_viewer.h>

#include <opencv2/core/core.hpp>
#include <opencv2/ml/ml.hpp>
#include <nabo/nabo.h>
#include "Eigen/Dense"

#include "segmatch/SegmentDefine.h"
#include "segmatch/SegMatchParam.hpp"
#include "segmatch/segmenters/segmenter.hpp"

using namespace pcl;
using namespace pcl::console;
using namespace pcl::visualization;
using namespace std;
using namespace cv;
using namespace Eigen;
using namespace Nabo;
using namespace segmatch;

template<typename T>
bool swap_if_gt(T& a, T& b) {
	if (a > b) {
		std::swap(a, b);
		return true;
	}
	return false;
}

PointXYZ calculateCentroid(PointCloud<PointXYZI>::Ptr& cloud);
cv::Mat calculateFeature(PointCloud<PointXYZI>::Ptr& cloud);
cv::Mat calculateFeatureDistance(const cv::Mat& feature1, const cv::Mat& feature2);

void displayPerformances(unsigned int tp, unsigned int tn, unsigned int fp, unsigned int fn);


class SegMatch
{
public:
	SegMatch(SegMatchParam segmatch_param);
	~SegMatch();

	void setSourceCloud(PointCloud<PointXYZI>::Ptr& cloud);
	void setTargetCloud(PointCloud<PointXYZI>::Ptr& cloud);

	int findCandidates(vector<Match>& matches, bool one_stage = true);
	int filterMatches(vector<Match>& matches, 
						vector<Match>& filtered_matches, 
						Eigen::Matrix4f& transformation);
	
	void trainRtrees(const Mat& features, const Mat& labels);
	void testRtrees(const Mat& features, const Mat& labels);
	void saveRtrees(const std::string& filename);

	const vector<Segment>* getSourceSegments(){return &m_source_segments;}
	const vector<Segment>* getTargetSegments(){return &m_target_segments;}

	void display(visualization::PCLVisualizer& p);

private:
	void filterCloud(PointCloud<PointXYZI>::Ptr& cloud);
	void describeCloud(PointCloud<PointXYZI>::Ptr& cloud, vector<PointIndices>& clusters, vector<Segment>& segments);
	void processCloud(PointCloud<PointXYZI>::Ptr& cloud, vector<Segment>& segments);
	


private:
	CvRTrees* m_rtrees;
	NNSearchF* m_nns;

	vector<Segment> m_source_segments;
	vector<Segment> m_target_segments;

	MatrixXf m_target_matrix;

	SegMatchParam param;

	std::unique_ptr<Segmenter> m_segmenter;
};


