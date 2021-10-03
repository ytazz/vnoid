#pragma once

namespace cnoid{
namespace vnoid{

/*
 3rd order butterworth filter
 */
struct Filter{
    double w, w2, w3;
    double u;
    double ydd, yd, y;

    // set cutoff frequency [Hz]
    void   SetCutoff(double f);

    double operator()(double _u, double dt);

    Filter();
};

}
}
