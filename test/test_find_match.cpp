#include <iostream>
#include <vector>
#include <string>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

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
    
    PointCloud<PointXYZI>::Ptr target(new PointCloud<PointXYZI>);
    PointCloud<PointXYZI>::Ptr source(new PointCloud<PointXYZI>);
    PCDReader reader;
    char filename[100];
    
    SegMatchParam param = GetSegMatchParam("../param/segmatch.yaml");
    SegMatch segmatch(param);

    int tgt_seg_cnt = 0;
    int src_seg_cnt = 0;

    printf("loading full map...\n");
    sprintf(filename, "%s%s-fullmap.pcd", param.file_directory.c_str(), param.map_name.c_str());
    reader.read(filename, *target);
    segmatch.setTargetCloud(target);

    visualization::PCLVisualizer *p = new visualization::PCLVisualizer(argc,argv,"find match");

    //get target segments form segmatch object
    const vector<Segment> *tgt_ptr = segmatch.getTargetSegments();
    
    //remove last view
    for(int i = 0; i < tgt_seg_cnt; i++)
        p->removePointCloud(string("target-")+to_string(i));    
    tgt_seg_cnt = 0;
    //draw target segments pointcloud
    for(auto seg = tgt_ptr->begin(); seg != tgt_ptr->end(); seg++)
    {   
        PointCloudColorHandlerCustom<PointXYZI> clr(seg->cloud.makeShared(), rand ()%255, rand ()%255, rand ()%255);
        p->addPointCloud(seg->cloud.makeShared(), clr, string("target-")+to_string(tgt_seg_cnt));
        tgt_seg_cnt++;
    }
    p->spinOnce();

    int j = 0;
    int true_match_cnt = 0;
    int false_match_cnt = 0;

    while(!p->wasStopped())
    {

        if(j >= param.source_number)
            break;
        printf("sourcemap[%d]\n", j);
        sprintf(filename, "%s%s-sourcemap-%d.pcd", param.file_directory.c_str(), param.map_name.c_str(), j);
        reader.read(filename, *source);
        segmatch.setSourceCloud(source);
        j++;

        vector<Match> matches;
        segmatch.findCandidates(matches, false);
        vector<Match> filtered_matches;
        Eigen::Matrix4f transformation;
        segmatch.filterMatches(matches, filtered_matches, transformation);
        cout<<transformation<<endl;
        for(auto m : filtered_matches)
        {
            PointXYZ p1 = m.centroid1;
            PointXYZ p2 = m.centroid2;

            float dist = sqrt(pow(p1.x-p2.x, 2)+
                                pow(p1.y-p2.y, 2)+
                                pow(p1.z-p2.z, 2));

            if(dist < param.min_distance) true_match_cnt++;
            else 
            {
                printf("false positive distanc: %f\n", dist);
                false_match_cnt++;
            }
        }
        printf("--true match count: %d, false match count: %d\n", true_match_cnt, false_match_cnt);


        //get source segments form Segmatch object  
        const vector<Segment> *src_ptr = segmatch.getSourceSegments();

        //display information
        printf("--cloud size: %lu segments size: %lu\n", source->size(), src_ptr->size());
        printf("--find %lu matches ", filtered_matches.size());
        for(auto m = filtered_matches.begin(); m != filtered_matches.end(); m++)
            printf("%4.4f, ", m->confidence);
        printf("\n");

        //remove last view
        for(int i = 0; i < src_seg_cnt; i++)
            p->removePointCloud(string("source-")+to_string(i));

        src_seg_cnt = 0;
        //draw new source segment pointcloud
        for(auto seg = src_ptr->begin(); seg != src_ptr->end(); seg++)
        {   
            PointCloud<PointXYZI>::Ptr cloud(new PointCloud<PointXYZI>);

            pcl::copyPointCloud(seg->cloud, *cloud);

            for(size_t i = 0; i < cloud->size(); i++)
                cloud->at(i).z += 50.0;
            
            PointCloudColorHandlerCustom<PointXYZI> clr(cloud, rand ()%255, rand ()%255, rand ()%255);
            p->addPointCloud(cloud, clr, string("source-")+to_string(src_seg_cnt));
            src_seg_cnt++;
        }

        //remove last arrows
        p->removeAllShapes();
        int lineid =0;
        //draw arrows of new matches
        for(auto match = filtered_matches.begin(); match != filtered_matches.end(); match++)
        {
            p->addLine<PointXYZ,PointXYZ>(PointXYZ(match->centroid1.x, match->centroid1.y, match->centroid1.z+50.0), 
                                        match->centroid2, 0, 1,0, string("line")+to_string(lineid));
            lineid++;
        }

        p->spinOnce (3000);
    }
    
            
    while(!p->wasStopped())
    {
        p->spinOnce (3000);
        // boost::this_thread::sleep (boost::posix_time::microseconds (3000));
    }
    return 0;
}
