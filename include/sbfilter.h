#pragma once

#include <sbtypes.h>

namespace Scenebuilder{;

/*
 1st order linear filter
 */
struct FilterLinear{
    real_t w;
    real_t u;
    real_t yd, y;

    void SetCutoff(real_t f);

    real_t operator()(real_t _u, real_t dt);

    FilterLinear();
};

/*
 3rd order butterworth filter
 */
struct FilterButterworth3{
    real_t w, w2, w3;
    real_t u;
    real_t ydd, yd, y;

    // set cutoff frequency [Hz]
    void   SetCutoff(real_t f);

    real_t operator()(real_t _u, real_t dt);

    FilterButterworth3();
};

}
