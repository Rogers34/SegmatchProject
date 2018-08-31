#include "MapConstruct.h"

int CMapConstruct::AddFrame(pcl::PointCloud<pcl::PointXYZI>::Ptr& points,
                            Eigen::Isometry3d& pose)
{
    //Whether robot move far enough to add point cloud into map
    if (m_PtContList.size() > 0)
    {
        Eigen::Isometry3d& last_pose = m_poseList.back();
        double dDist = sqrt(pow((last_pose(0,3)-pose(0,3)),2) +
                            pow((last_pose(1,3)-pose(1,3)),2));
        // std::cout<<"[CMapConstruct::AddFrame]: "<<dDist<<" meters already go."<<std::endl;
        if (dDist < m_dMapInterval)
        {
            return -1;
        }
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr temp(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::transformPointCloud(*points, *temp, pose.matrix());
    m_PtList += (*temp);
    m_PtContList.push_back(temp->size());
    m_poseList.push_back(pose);

    if (m_PtContList.size() > m_nLatestCapacity)
    {
        // std::cout<<"ptsize: "<<m_PtContList.size()<<std::endl;
        m_PtList.erase(m_PtList.begin(), m_PtList.begin()+m_PtContList.back());
        m_PtContList.erase(m_PtContList.begin(),m_PtContList.begin()+1);
        m_poseList.erase(m_poseList.begin(), m_poseList.begin()+1);
    }

    return m_PtList.size();
}

int CMapConstruct::GetMap(pcl::PointCloud<pcl::PointXYZI>::Ptr& points)
{
    (*points) = m_PtList;

    return points->size();
}
