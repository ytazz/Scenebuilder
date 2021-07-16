#include <sbfilter.h>

namespace Scenebuilder{;

Filter::Filter(){
    ydd = 0.0;
    yd  = 0.0;
    y   = 0.0;
    w   = 1.0;
    w2  = w*w;
    w3  = w*w2;
}

void Filter::SetCutoff(real_t f){
    w   = 2.0*3.1415*f;
    w2  = w*w;
    w3  = w*w2;
}

real_t Filter::operator()(real_t _u, real_t dt){
    u    = _u;
    y   += yd*dt;
    yd  += ydd*dt;
    ydd += (-2.0*w*ydd - 2.0*w2*yd - w3*y + w3*u)*dt;

    return y;
}

}

