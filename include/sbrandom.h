#pragma once

#include <sbtypes.h>

namespace Scenebuilder{;

class SamplerImpl;

class Sampler{
	SamplerImpl* impl;

public:

	void   Seed        (int    _seed);
	bool   SampleBool  ();
	int    SampleInt   (int    _min, int    _max);
	real_t SampleReal  (real_t _min, real_t _max);
	real_t SampleNormal();

	 Sampler();
	~Sampler();
};

}
