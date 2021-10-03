#include "filter.h"

namespace cnoid{
namespace vnoid{

const double pi = 3.14159265358979;

Filter::Filter(){
    ydd = 0.0;
    yd  = 0.0;
    y   = 0.0;
    w   = 1.0;
    w2  = w*w;
    w3  = w*w2;
}

void Filter::SetCutoff(double f){
    w   = 2.0*pi*f;
    w2  = w*w;
    w3  = w*w2;
}

double Filter::operator()(double _u, double dt){
    u    = _u;
    y   += yd*dt;
    yd  += ydd*dt;
    ydd += (-2.0*w*ydd - 2.0*w2*yd - w3*y + w3*u)*dt;

    return y;
}

}
}
