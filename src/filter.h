#pragma once

namespace cnoid{
namespace vnoid{

/**
 3rd order butterworth filter
 **/
struct Filter{
    /// filter coefficients
    double w, w2, w3;

    /// internal storage of input signal value
    double u;

    /// internal storage of output signal value, up to its second derivative
    double ydd, yd, y;

    /**
     @brief set cutoff frequency
     @param f cufoff frequency in [Hz]
     **/
    void   SetCutoff(double f);

    /**
     @brief filters the signal
     @param  _u  input signal
     @param  dt  sampling period
     @return     filtered output
     **/
    double operator()(double _u, double dt);

    Filter();
};

}
}
