#pragma once
#include <vector>
#include <string>
#include <stdio.h>

//typedef unsigned long long uint64_t;
struct BlockTrj
{
	uint64_t time;
	unsigned char laserId;
	uint64_t fileDataIdx;
	double pos[3];
	double q[4];
};

struct HeaderTrj
{
	unsigned char headerlen;
	unsigned char laserNum;
	std::vector<std::string> vLaserName;
	uint64_t blockNum;
};

struct _iobuf;
class TrjFile
{
public:
	TrjFile();

	~TrjFile();

	int open(const char *fileName);

	int writeHeader(std::vector<std::string> &vLaserName);

	int writeBlock(BlockTrj &line);

	void close();

private:
//	_iobuf  *fp_;
    FILE  *fp_;
};


class TrjFileParse
{
public:
	TrjFileParse();

	~TrjFileParse();

	int open(const char *fileName);

	int readHeader(HeaderTrj &header);

	int readBlock(BlockTrj &line);

	void close();

private:
//	_iobuf  *fp_;
    FILE  *fp_;
};

