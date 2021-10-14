#include <sbfilter.h>

namespace Scenebuilder{;

const real_t pi = 3.14159265368979;

///////////////////////////////////////////////////////////////////////////////////////////////////

FilterLinear::FilterLinear(){
    yd  = 0.0;
    y   = 0.0;
    w   = 1.0;
}

void FilterLinear::SetCutoff(real_t f){
    w   = 2.0*pi*f;
}

real_t FilterLinear::operator()(real_t _u, real_t dt){
    u    = _u;
    yd   = -w*(y - u);

    real_t a = exp(-w*dt);
    y    = a*y + (1.0 - a)*u;
   
    return y;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

FilterButterworth3::FilterButterworth3(){
    ydd = 0.0;
    yd  = 0.0;
    y   = 0.0;
    w   = 1.0;
    w2  = w*w;
    w3  = w*w2;
}

void FilterButterworth3::SetCutoff(real_t f){
    w   = 2.0*pi*f;
    w2  = w*w;
    w3  = w*w2;
}

real_t FilterButterworth3::operator()(real_t _u, real_t dt){
    u    = _u;
    y   += yd*dt;
    yd  += ydd*dt;
    ydd += (-2.0*w*ydd - 2.0*w2*yd - w3*y + w3*u)*dt;

    return y;
}

}

