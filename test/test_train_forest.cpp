#include <iostream>
#include <vector>
#include <string>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <nabo/nabo.h>

#include <opencv2/core/eigen.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/ml/ml.hpp>

#include "segmatch/SegMatch.h"

#define DISPLAY 0

using namespace std;
using namespace cv;
using namespace pcl;

using namespace Nabo;
using namespace Eigen;


int main (int argc, char *argv[])
{
    PointCloud<PointXYZI>::Ptr target(new PointCloud<PointXYZI>);
    PointCloud<PointXYZI>::Ptr source(new PointCloud<PointXYZI>);
        
    char filename[1000];    
    PCDReader reader;

    printf("------------------------------------------------------------\n");
    SegMatchParam param= GetSegMatchParam("../param/segmatch.yaml");
    SegMatch segmatch(param);

    Mat features;
    Mat labels;


    int source_cnt = 0;

    // load full map
    printf("loading full map...\n");
    sprintf(filename, "%s%s-fullmap.pcd", param.file_directory.c_str(), param.map_name.c_str());
    reader.read(filename, *target);
    segmatch.setTargetCloud(target);

    const vector<Segment> *target_segment = segmatch.getTargetSegments();
    PointCloud<PointXYZ>::Ptr target_segment_centroid(new PointCloud<PointXYZ>);
    vector<size_t> target_segment_index;

    for(int i = 0; i < target_segment->size(); i++)
        target_segment_centroid->push_back(target_segment->at(i).centroid);

    int total_pos_label = 0;
    int total_neg_label = 0;

    ///////////////////////////////////////////////////////////////////////
    #if DISPLAY
    // draw target cloud
    int tgt_seg_cnt = 0;
    int src_seg_cnt = 0;
    visualization::PCLVisualizer *view = new visualization::PCLVisualizer(argc,argv,"rtrees train");
    for(auto seg = target_segment->begin(); seg != target_segment->end(); seg++)
    {   
        PointCloudColorHandlerCustom<PointXYZI> clr(seg->cloud.makeShared(), rand ()%255, rand ()%255, rand ()%255);
        view->addPointCloud(seg->cloud.makeShared(), clr, string("target-")+to_string(tgt_seg_cnt));
        tgt_seg_cnt++;
    }
    view->spinOnce();
    #endif
    //////////////////////////////////////////////////////////////////////////

	int n_knn = (param.knn_number <= target_segment->size()) ? param.knn_number : target_segment->size();

    for(int id = 0; id < param.source_number; id++)
    {
        printf("sourcemap[%d]\n", id);
        sprintf(filename, "%s%s-sourcemap-%d.pcd", param.file_directory.c_str(), param.map_name.c_str(), id);
        reader.read(filename, *source);
        segmatch.setSourceCloud(source);

        const vector<Segment> *source_segment = segmatch.getSourceSegments();

        //////////////////////////////////////////////////////////////////////
        #if DISPLAY
        // draw source cloud
        for(int i_seg = 0; i_seg < src_seg_cnt; i_seg++)
            view->removePointCloud(string("source-")+to_string(i_seg));
        src_seg_cnt = 0;
        for(auto seg = source_segment->begin(); seg != source_segment->end(); seg++)
        {   
            PointCloud<PointXYZI>::Ptr cloud(new PointCloud<PointXYZI>);

            pcl::copyPointCloud(seg->cloud, *cloud);

            for(size_t i = 0; i < cloud->size(); i++)
                cloud->at(i).z += 50.0;
            
            PointCloudColorHandlerCustom<PointXYZI> clr(cloud, rand ()%255, rand ()%255, rand ()%255);
            view->addPointCloud(cloud, clr, string("source-")+to_string(src_seg_cnt));
            src_seg_cnt++;
        }
        view->spinOnce();
        #endif
        ////////////////////////////////////////////////////////////////////////

        vector<Match> matches;
        // knn only
        segmatch.findCandidates(matches);
		
        for(size_t i = 0; i < source_segment->size(); i++)
        {
            int loop_neg_label = 0;
            int loop_pos_label = 0;

            for(size_t j = 0; j < n_knn; j++)
            {
                size_t idx = i * n_knn + j;
				//calculate distance between source segment centroid and target segment centroid
                float dist = sqrt(pow(matches[idx].centroid1.x - matches[idx].centroid2.x, 2)+
                                    pow(matches[idx].centroid1.y - matches[idx].centroid2.y, 2)+
                                    pow(matches[idx].centroid1.z - matches[idx].centroid2.z, 2));

                if(dist < param.min_distance)   // true match found
                {
                    Mat diff = calculateFeatureDistance(matches[idx].feature1, matches[idx].feature2);
                    features.push_back(diff);
                    labels.push_back(1.0f);
                    total_pos_label++;
                    loop_pos_label++;

                    ////////////////////////////////////////////////////////////
                    #if DISPLAY
                    //draw arrows of new matches
                    view->removeAllShapes();
                    view->addLine<PointXYZ,PointXYZ>(PointXYZ(matches[idx].centroid1.x, 
                                                        matches[idx].centroid1.y, 
                                                        matches[idx].centroid1.z+50.0), 
                                                    matches[idx].centroid2, 0, 1, 0, string("line"));
                    view->spinOnce(1000);
                    #endif
                    ////////////////////////////////////////////////////////////
                }
                else if(loop_neg_label < 50)    // false match found
                {
                    Mat diff = calculateFeatureDistance(matches[idx].feature1, matches[idx].feature2);
                    features.push_back(diff);
                    labels.push_back(0.0f);
                    total_neg_label++;
                    loop_neg_label++;
                }
            }
		}
    }
    printf("pos_label: %d neg_label: %d\n", total_pos_label, total_neg_label);
    segmatch.trainRtrees(features, labels);
	segmatch.testRtrees(features, labels);
	// segmatch.trainRtrees(features.rowRange(0, features.rows/2), 
	// 						labels.rowRange(0, labels.rows/2));
    // segmatch.testRtrees(features.rowRange(features.rows/2, features.rows), 
	// 						labels.rowRange(labels.rows/2, labels.rows));
    printf("\n");

    segmatch.saveRtrees(param.output_rtrees_name);

    return 0;
}