#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/time.h> //fps calculations
#include <pcl/io/pcd_io.h>
#include "hdl_grabber_.h"
#include "vlp_grabber_.h"
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/image_viewer.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/boost.h>
#include <pcl/visualization/mouse_event.h>
#include <vector>
#include <string>
#include <boost/algorithm/string.hpp>
#include <typeinfo>

// using namespace std;
// using namespace pcl;
// using namespace pcl::console;
// using namespace pcl::visualization;


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointType>
class SimpleVLPViewer
{
public:
	typedef pcl::PointCloud<PointType> Cloud;
	typedef typename Cloud::ConstPtr CloudConstPtr;
	typedef typename Cloud::Ptr CloudPtr;

	SimpleVLPViewer (pcl::Grabber& grabber):
	grabber_ (grabber)
	{
		m_bIsNewData =  false;
        m_nReadMode = 0;
	}

	void cloud_callback (const CloudConstPtr& cloud)
	{
		boost::mutex::scoped_lock lock(cloud_mutex_);
		cloud_ = cloud;
		m_bIsNewData = true;
        // pcl::console::print_info("Frame id: %d, Points cont: %d Time stamp: %llu\n", cloud->header.seq, cloud->size(), cloud->header.stamp);
	}

    void setMode(int nMode)
    {
        m_nReadMode = nMode;
    }

	void start()
	{
        if (m_nReadMode == 1)
        {
            ((pcl::HDLGrabber*)(&grabber_))->pause();
        }
		boost::function<void 
			(const CloudConstPtr&)> cloud_cb = boost::bind (&SimpleVLPViewer::cloud_callback, this, _1);
        cloud_connection = grabber_.registerCallback (cloud_cb);
		grabber_.start ();
	}

    int nextFrame(CloudConstPtr& cloud)
    {
        ((pcl::HDLGrabber*)(&grabber_))->resume();
        int nRepeat = 0;
        while (1)
        {
            int nRt = GetData(cloud);
            if (nRt == 0)
            {
                nRepeat++;
                usleep(10000);
                if (nRepeat >= 100)
                {
                    ((pcl::HDLGrabber*)(&grabber_))->pause();
                    return 0;
                }
                else
                {
                    continue;
                }
            }
            else
            {
                ((pcl::HDLGrabber*)(&grabber_))->pause();

                return 1;
            }
        }


        ((pcl::HDLGrabber*)(&grabber_))->pause();

        return 0;
    }

	void stop()
	{
		grabber_.stop ();
		cloud_connection.disconnect ();
	}

	int GetData(CloudConstPtr& cloud)
	{
		if (cloud_mutex_.try_lock ())
		{
			if (!m_bIsNewData)
			{
				cloud_mutex_.unlock ();
				return 0;
			}
			cloud.swap(cloud_);
			m_bIsNewData = false;
			cloud_mutex_.unlock ();
			return 1;
		}
		else
		{
			return 0;
		}
		return 0;
	}

    int m_nReadMode;    //0:On line  1:frame by frame
    boost::signals2::connection cloud_connection;
	pcl::Grabber& grabber_;
	boost::mutex cloud_mutex_;
	CloudConstPtr cloud_;
	bool m_bIsNewData;
};

//
//int main(int argc, char ** argv)
//{
//	std::string pcapFile;
//
//	parse_argument (argc, argv, "-pcapFile", pcapFile);
//
//	VLPGrabber grabber(pcapFile);
//
//	SimpleVLPViewer<PointXYZI> v(grabber);
//	v.start();
//
//	pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
//
//	while(1)
//	{
//		PointCloud<PointXYZI>::ConstPtr ptCloudPt;
//		int nRt = v.GetData(ptCloudPt);
//		if (nRt == 0)
//		{
//			Sleep(10);
//			continue;
//		}
//
//		viewer.showCloud(ptCloudPt);
//	}
//
//	return (0);
//}
