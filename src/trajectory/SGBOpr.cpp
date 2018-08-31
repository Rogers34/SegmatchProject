#include "SGBOpr.h"
#include<iostream>
#include<fstream>
#include<string.h>
#include<iomanip>

using namespace std;

CSgbOpr::CSgbOpr()
{
    m_nLastSearchInd = 0;
}

int CSgbOpr::ReadLog(std::string szLogPath)
{
    ifstream fs(szLogPath,ios_base::binary);
    int nDataLen = sizeof(IMUData);
    vector<char> buf;
    buf.resize(nDataLen);
    IMUData DataTemp;
    while (fs.read(buf.data(), nDataLen))
    {
        memcpy(&DataTemp, buf.data(), nDataLen);
        m_ImuData.push_back(DataTemp);
    }

    m_HeadAcc.reserve(m_ImuData.size());
    m_HeadAcc.push_back(0.0);
    for (unsigned int i = 0; i+1 < m_ImuData.size(); i++)
    {
        m_HeadAcc.push_back(m_HeadAcc.back()+m_ImuData[i].gyro[2]);
    }

    m_SysTime.reserve(m_ImuData.size());
    uint64_t nUscd = 0;
    for (unsigned int i = 0; i < m_ImuData.size(); i++)
    {
        uint64_t nWeek = m_ImuData[i].week*(uint64_t)(7*24*3600*1000000ll);
        uint64_t nSec = m_ImuData[i].sec*1000000.0;
        uint64_t nTime = nWeek + nSec;
        m_SysTime.push_back(nTime);
//        cout << nTime << endl;
    }
    cout << m_SysTime.front() << endl;
    cout << m_SysTime.back() << endl;

    ofstream fs_out("ImuData.txt");
    for (unsigned int i = 0; i < m_ImuData.size(); i++)
    {
        fs_out <<
                  m_ImuData[i].acc[0] << "," <<
                                         m_ImuData[i].acc[1] << "," <<
                                         m_ImuData[i].acc[2] << "," <<
                                         m_ImuData[i].gyro[0] << "," <<
                                         m_ImuData[i].gyro[1] << "," <<
                                         m_ImuData[i].gyro[2] << "," <<
                                         m_ImuData[i].sec << "," <<
                                         m_ImuData[i].utcsec << "," <<
                                         m_ImuData[i].utcweek << "," <<
                                         m_ImuData[i].week << "," <<
                                         std::endl;
    }

    return m_ImuData.size();
}

uint64_t CSgbOpr::GetDataByTime(uint64_t nTIme, IMUData& Data)
{
    for (uint64_t i = m_nLastSearchInd; i+1 < m_SysTime.size(); i++)
    {
        if (m_SysTime[i] <= nTIme && m_SysTime[i+1] >= nTIme)
        {
            m_nLastSearchInd = i;
            Data = m_ImuData[i];
            return m_SysTime[i];
        }
    }

    return 0;
}

int CSgbOpr::GetHeadingChange(uint64_t nTIme, double& dHeadChange)
{
    double dAngleBefor = m_HeadAcc[m_nLastSearchInd];
    int nCurInd = -1;
    for (uint64_t i = m_nLastSearchInd; i+1 < m_SysTime.size(); i++)
    {
        if (m_SysTime[i] <= nTIme && m_SysTime[i+1] >= nTIme)
        {
            m_nLastSearchInd = i;
            nCurInd = i;
            break;
        }
    }
    if (nCurInd < 0)
    {
        return -1;
    }
    dHeadChange = m_HeadAcc[nCurInd] - dAngleBefor;

    return 1;
}
