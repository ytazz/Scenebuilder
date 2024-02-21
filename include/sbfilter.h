#pragma once

#include <sbtypes.h>

namespace Scenebuilder{;

/*
 1st order linear filter
 */
class Filter{
public:
    struct Type{
        enum{
            None,
            FirstOrderLPF,
            FirstOrderLPF2,
            Butterworth3,
        };
    };

    int     type;
    real_t  w, w2, w3;
    real_t  u;
    real_t  ydd, yd, y;
    bool    first;
    real_t  yd_max;

public:
    void SetCutoff(real_t f);

    real_t operator()(real_t _u, real_t dt);

    Filter();
};

}
