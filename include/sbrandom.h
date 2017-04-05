#pragma once

#include <sbtypes.h>

namespace Scenebuilder{;

class SamplerImpl;

class Sampler{
	SamplerImpl* impl;

public:

	void   Seed      (int    _seed);
	int    SampleInt (int    _min, int    _max);
	real_t SampleReal(real_t _min, real_t _max);

	 Sampler();
	~Sampler();
};

}
