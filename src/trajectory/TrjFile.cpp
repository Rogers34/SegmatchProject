#include "TrjFile.h"
#include "stdio.h"
#include <string.h>

TrjFile::TrjFile():fp_(0)
{
}

TrjFile::~TrjFile()
{
	if(fp_)
		close();
}

int TrjFile::open(const char *fileName)
{
	if (fp_)
	{
		fclose(fp_);
	}

	fp_ = fopen(fileName, "wb");
	//fp_ = fopen(fileName, "w");
	if (!fp_)
		return -1;
	return 0;
}

int TrjFile::writeHeader(std::vector<std::string> &vLaserName)
{
	char LaserName[100];
	unsigned char laserNum = vLaserName.size();
	unsigned char headerlen = laserNum * 2;

	for (size_t i = 0; i < laserNum; i++)
	{
		headerlen += vLaserName[i].length();
		headerlen++;
	}

	if (fp_)
	{
		fwrite(&headerlen, 1, 1, fp_);
		fwrite(&laserNum, 1, 1, fp_);
		for (size_t i = 0; i < laserNum; i++)
		{
			std::string &name = vLaserName[i];
			size_t len = name.length() + 1;
			memcpy(LaserName, name.c_str(), len);
			LaserName[len - 1] = '\0';
			fwrite(LaserName, len, 1, fp_);
		}
		uint64_t blockNum = 0;
		fwrite(&blockNum, sizeof(blockNum), 1, fp_);
		return 0;
	}
	return -1;
}


int TrjFile::writeBlock(BlockTrj &line)
{
	if (fp_)
	{
		fwrite(&line, sizeof(line), 1, fp_);
		/*fprintf(fp_, "%lld %f %f %f %f %f %f %f\n", line.time, 
			line.pos[0], line.pos[1], line.pos[2],
			line.q[0], line.q[1], line.q[2], line.q[3]);*/
		return 0;
	}
	return -1;
}

void TrjFile::close()
{
	if (fp_)
	{
		fclose(fp_);
		fp_ = 0;
	}
}

TrjFileParse::TrjFileParse() :fp_(0)
{

}

TrjFileParse::~TrjFileParse()
{
	if (fp_)
		close();
}

int TrjFileParse::open(const char *fileName)
{
	if (fp_)
	{
		fclose(fp_);
	}

	fp_ = fopen(fileName, "rb");
	if (!fp_)
		return -1;
	return 0;
}

int TrjFileParse::readHeader(HeaderTrj &header)
{
	if (fp_)
	{
		fseek(fp_, 0, SEEK_END); //定位到文件末 
//		long long nFileLen = _ftelli64(fp_); //文件长度
        uint64_t nFileLen = ftell(fp_);

		fseek(fp_, 0, SEEK_SET);

		fread(&header.headerlen, 1, 1, fp_);
		fread(&header.laserNum, 1, 1, fp_);

		char tmp;
		int strcount = 0;
		for (size_t i = 0, count = 0; i < header.laserNum; i++)
		{
			std::string name;
			while (1)
			{
				fread(&tmp, 1, 1, fp_);
				strcount++;
				if (tmp == '\0')
				{
					break;
				}
				name += tmp;
				count++;
				if (count >= header.headerlen)
				{
					return -2;
				}
			}
			header.vLaserName.push_back(name);
		}
		nFileLen -= header.headerlen;
		fread(&header.blockNum, sizeof(uint64_t), 1, fp_);
		header.blockNum = nFileLen / sizeof(BlockTrj);
		return 0;
	}
	return -1;
}

int TrjFileParse::readBlock(BlockTrj &line)
{
	if (fp_)
	{
		if(fread(&line, sizeof(line), 1, fp_) == 0)
			return -1;
		else
			return 0;
	}
	return -2;
}

void TrjFileParse::close()
{
	if (fp_)
	{
		fclose(fp_);
		fp_ = 0;
	}
}
