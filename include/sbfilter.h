#pragma once

#include <sbtypes.h>

namespace Scenebuilder{;

/*
 3rd order butterworth filter
 */
struct Filter{
    real_t w, w2, w3;
    real_t u;
    real_t ydd, yd, y;

    // set cutoff frequency [Hz]
    void   SetCutoff(real_t f);

    real_t operator()(real_t _u, real_t dt);

    Filter();
};

}
