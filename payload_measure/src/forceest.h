#ifndef FORCEEST_H
#define FORCEEST_H
#include "ukf.h"
#include "iostream"
#include "lpf2.h"




enum state{
    thetap=0,
    omegap,
    ac_x,
    ac_z,
    ap_x,
    ap_z,
    FF_x,
    FF_z,
    FL_x,
    FL_z,
    statesize
};

enum measurement{
    mthetap = 0,
    momegap,
    mac_x,
    mac_z,
    mFF_x,
    mFF_z,
    measurementsize
};


class forceest : public ukf
{

public:
  lpf2 *lpf;
forceest(int x, int y) : ukf(x,y){
  last_omega_p = 0;
    lpf = new lpf2(6,0.02);
}

Eigen::MatrixXd dynamics(Eigen::MatrixXd sigma_state);
double last_omega_p ;
private:


};

#endif // FORCEEST_H
