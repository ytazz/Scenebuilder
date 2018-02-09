#include <sbrandom.h>

#include <boost/random.hpp>
#include <boost/random/uniform_int_distribution.hpp>
#include <boost/random/uniform_real_distribution.hpp>

namespace Scenebuilder{;

///////////////////////////////////////////////////////////////////////////////////////////////////

class SamplerImpl{
public:
	boost::random::mt19937                     rng;
	boost::random::uniform_int_distribution<>  uniInt;
	boost::random::uniform_real_distribution<> uniReal;

public:
	void   Seed      (int    _seed);
	int    SampleInt (int    _min, int    _max);
	real_t SampleReal(real_t _min, real_t _max);

};

void SamplerImpl::Seed(int _seed){
	rng.seed(_seed);
}

int SamplerImpl::SampleInt (int _min, int _max){
	if(_min >= _max)
		return _min;

	uniInt.param(boost::random::uniform_int_distribution<>::param_type(_min, _max));
	return uniInt(rng);
}

real_t SamplerImpl::SampleReal(real_t _min, real_t _max){
	if(_min >= _max)
		return _min;

	uniReal.param(boost::random::uniform_real_distribution<>::param_type(_min, _max));
	return uniReal(rng);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

Sampler::Sampler(){
	impl = new SamplerImpl();
}

Sampler::~Sampler(){
	delete impl;
}

void Sampler::Seed(int _seed){
	impl->Seed(_seed);
}

bool Sampler::SampleBool(){
	return (bool)SampleInt(0, 1);
}

int Sampler::SampleInt(int _min, int _max){
	return impl->SampleInt(_min, _max);
}

real_t Sampler::SampleReal(real_t _min, real_t _max){
	return impl->SampleReal(_min, _max);
}

}
