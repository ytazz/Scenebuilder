#include <sbfilter.h>

namespace Scenebuilder{;

const real_t pi = 3.14159265368979;

///////////////////////////////////////////////////////////////////////////////////////////////////

Filter::Filter(){
    u      = 0.0;
    ydd    = 0.0;
    yd     = 0.0;
    y      = 0.0;
    w      = 1.0;
    w2     = w*w;
    w3     = w*w2;
    yd_max = 0.0;

    type  = Type::FirstOrderLPF;
    first = true;
}

void Filter::SetCutoff(real_t f){
    w   = 2.0*pi*f;
    w2  = w*w;
    w3  = w*w2;
}

real_t Filter::operator()(real_t _u, real_t dt){
    if(first){
        y = _u;
        first = false;
    }

    u = _u;

    if(type == Type::None){
        y = u;
    }
    if(type == Type::FirstOrderLPF){
        yd   = -w*(y - u);

        if(yd_max > 0.0){
            yd = std::min(std::max(-yd_max, yd), yd_max);
        }

        y += yd*dt;
    }
    if(type == Type::FirstOrderLPF2){
        real_t a = exp(-w*dt);
        y    = a*y + (1.0 - a)*u;
    }
    if(type == Type::Butterworth3){
        y   += yd*dt;
        yd  += ydd*dt;

        if(yd_max > 0.0){
            yd = std::min(std::max(-yd_max, yd), yd_max);
        }

        ydd += (-2.0*w*ydd - 2.0*w2*yd - w3*y + w3*u)*dt;
    }
   
    return y;
}

}

