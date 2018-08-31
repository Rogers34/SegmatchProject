#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/core/core.hpp>

namespace segmatch
{

struct Match
{
    int id1;    //source segment id
    int id2;    //target segment id
    pcl::PointXYZ centroid1;
    pcl::PointXYZ centroid2;
    cv::Mat feature1;
    cv::Mat feature2;
    float confidence;
};

struct Segment
{
    int id;
    int frame;
    pcl::PointCloud<pcl::PointXYZI> cloud;
    cv::Mat feature;
    pcl::PointXYZ centroid;
    uint64_t time_stamp;
};


}//end of namespace segmatch
