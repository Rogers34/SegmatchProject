#include <iostream>
#include <vector>
#include <string>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

#include <pcl/common/transforms.h> 


#include <nabo/nabo.h>

#include <opencv2/core/eigen.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/ml/ml.hpp>

#include "segmatch/SegMatch.h"

using namespace std;
using namespace cv;
using namespace pcl;

using namespace Nabo;
using namespace Eigen;


int main (int argc, char *argv[])
{
    if(argc < 2)return -1;
    SegMatchParam param = GetSegMatchParam("../param/segmatch.yaml");
    
    PointCloud<PointXYZI>::Ptr target(new PointCloud<PointXYZI>);
    PointCloud<PointXYZI>::Ptr source(new PointCloud<PointXYZI>);
    PointCloud<PointXYZI>::Ptr source0(new PointCloud<PointXYZI>);
    PointCloud<PointXYZI>::Ptr source1(new PointCloud<PointXYZI>);
    PCDReader reader;
    string file_name;
    file_name = param.file_directory+param.map_name+"-fullmap.pcd";
    reader.read(file_name, *target);

    SegMatch segmatch(param);
    segmatch.setTargetCloud(target);

    Eigen::Isometry3f t0 = Eigen::Isometry3f::Identity();
    t0.pretranslate(Eigen::Vector3f(0.0, 0.0, 50.0));
    // t0.rotate(Eigen::AngleAxisf(3.1415926/4, Eigen::Vector3f::UnitZ ()));
    cout<<t0.matrix()<<endl;

    file_name = param.file_directory+param.map_name+"-sourcemap-"+argv[1]+".pcd";
    reader.read(file_name, *source);

    transformPointCloud(*source, *source0, t0);

    segmatch.setSourceCloud(source0);
    
    vector<Match> matches;
    segmatch.findCandidates(matches, false);
    
    vector<Match> filtered_matches;
    Eigen::Matrix4f transformation;
    segmatch.filterMatches(matches, filtered_matches, transformation);
    transformPointCloud(*source0, *source1, transformation);

    cout<<transformation.matrix()<<endl;

    visualization::PCLVisualizer p;
    PointCloudColorHandlerCustom<PointXYZI> h0(target, 255, 255, 255);
    p.addPointCloud(target, h0, string("target"));
    // PointCloudColorHandlerCustom<PointXYZI> h1(source, 200, 50, 50);
    // p.addPointCloud(source, h1, string("source"));
    PointCloudColorHandlerCustom<PointXYZI> h2(source0, 50, 50, 200);
    p.addPointCloud(source0, h2, string("source0"));
    PointCloudColorHandlerCustom<PointXYZI> h3(source1, 50, 200, 50);
    p.addPointCloud(source1, h3, string("source1"));
    p.setPointCloudRenderingProperties(PCL_VISUALIZER_POINT_SIZE, 2, "source1");
    

    while(!p.wasStopped())
    {
        p.spinOnce(1000);
    }

    return 0;
}
