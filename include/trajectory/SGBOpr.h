#ifndef _SGB_OPR_H_
#define _SGB_OPR_H_

#include <string>
#include <vector>

//struct IMUData
//{
//    unsigned int week;//系统时间
//    double sec;
//    float gyro[3];
//    float acc[3];
//    float mag[3];
//    unsigned int utcweek;//传感器时间
//    double utcsec;
//};

struct IMUData
{
    unsigned int week;//系统时间
    double sec;
    float gyro[3];
    float acc[3];
    float mag[3];
    unsigned int utcweek;//传感器时间
    double utcsec;
};

class CSgbOpr
{
public:
    CSgbOpr();
    int ReadLog(std::string szLogPath);
    uint64_t GetDataByTime(uint64_t nTIme, IMUData& Data);
    int GetHeadingChange(uint64_t nTIme, double& dHeadChange);

    std::vector<IMUData> m_ImuData;
    uint64_t m_nLastSearchInd;
    std::vector<uint64_t> m_SysTime;
    std::vector<double> m_HeadAcc;
};

#endif
