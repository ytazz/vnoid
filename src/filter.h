#pragma once

namespace cnoid{
namespace vnoid{

/**
  filter
 **/
struct Filter{
    struct Type{
        enum{
            None,
            FirstOrderLPF,
            FirstOrderLPF2,
            Butterworth3,   ///< 3rd order butterworth 
        };
    };

    /// type of filter
    int  type;

    /// filter coefficients
    double w, w2, w3;

    /// internal storage of input signal value
    double u;

    /// internal storage of output signal value, up to its second derivative
    double ydd, yd, y;

    /// limit of time derivative. yd is clamped to [-yd_max, yd_max]. no limit if yd_max == 0 (default)
    double yd_max;

    /// internal flag to mark first call
    bool   first;
    

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
