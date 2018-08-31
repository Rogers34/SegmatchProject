#pragma once
#include <vector>
#include <string>
#include <stdio.h>
#include "TrjFile.h"
#include <pcl/common/transforms.h>

class CTrjLogReader
{
public:
    CTrjLogReader();
    int ReadLog(std::string szLogPath);
    int GetPoseByTime(uint64_t nTime, Eigen::Isometry3d& pose);

private:
    int m_nSearchInd;
    std::vector<BlockTrj> m_poses;
};

