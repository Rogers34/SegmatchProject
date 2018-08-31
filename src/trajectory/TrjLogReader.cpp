#include "TrjLogReader.h"

CTrjLogReader::CTrjLogReader()
{
    m_nSearchInd = 0;
}

int CTrjLogReader::ReadLog(std::string szLogPath)
{
    TrjFileParse trj;
    trj.open(szLogPath.c_str());
    HeaderTrj header;
    trj.readHeader(header);
    while(1)
    {
        BlockTrj poseT;
        if (trj.readBlock(poseT) < 0)
        {
            break;
        }
        m_poses.push_back(poseT);
    }
    std::cout << "Poses cont: " << m_poses.size() << std::endl;
    std::cout << "Start pose" << m_poses.front().pos[0] << "," <<
                 m_poses.front().pos[1] << "," <<
                 m_poses.front().pos[2] << "," << std::endl;
    std::cout << "End pose" << m_poses.back().pos[0] << "," <<
                 m_poses.back().pos[1] << "," <<
                 m_poses.back().pos[2] << "," << std::endl;

    return m_poses.size();
}

int CTrjLogReader::GetPoseByTime(uint64_t nTime, Eigen::Isometry3d& pose)
{
    uint64_t lidar_time_stamp = nTime;
    int nSearchT = -1;
    for (unsigned int i = m_nSearchInd; i+1 < m_poses.size(); i++)
    {
        if(m_poses[i].time <= lidar_time_stamp &&
                m_poses[i+1].time >= lidar_time_stamp)
        {
                
            m_nSearchInd = i;
            nSearchT = i;
            break;
        }
    }
    if (nSearchT < 0 || nSearchT >= m_poses.size())
    {
        return -1;
    }
    BlockTrj& pose_now = m_poses[nSearchT];
    Eigen::Quaterniond q(pose_now.q[0], pose_now.q[1], pose_now.q[2], pose_now.q[3]);
    Eigen::Isometry3d tran;
    pose = q;
    pose(0, 3) = pose_now.pos[0];
    pose(1, 3) = pose_now.pos[1];
    pose(2, 3) = pose_now.pos[2];


    return 1;
}
