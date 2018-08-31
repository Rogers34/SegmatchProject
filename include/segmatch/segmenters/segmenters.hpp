#ifndef SEGMATCH_SEGMENTERS_HPP_
#define SEGMATCH_SEGMENTERS_HPP_

#include "segmatch/segmenters/euclidean_segmenter.hpp"
#include "segmatch/segmenters/region_growing_segmenter.hpp"
#include "segmatch/segmenters/segmenter.hpp"
#include "segmatch/SegMatchParam.hpp"


static std::unique_ptr<Segmenter> create_segmenter(const SegMatchParam& param) 
{
	std::unique_ptr<Segmenter> sgmtr;
	if (param.segmenter_type == "RegionGrowingSegmenter") 
	{
		sgmtr = std::unique_ptr<Segmenter>(new RegionGrowingSegmenter(param));
	} 
	else if (param.segmenter_type == "EuclideanSegmenter") 
	{
		sgmtr = std::unique_ptr<Segmenter>(new EuclideanSegmenter(param));
	} 
	else 
	{
		std::cout << "The segmenter " << param.segmenter_type << " was not implemented.";
	}
	return sgmtr;
}


#endif // SEGMATCH_SEGMENTERS_HPP_
