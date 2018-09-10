#include "forceest.h"

#define L 1.2     //length of object
#define pi 3.14159 //pi
#define g  9.81    //gravity
#define r  0.8   //0.5length of object
#define a 0.008   //width of object
#define l  0.4   //  length of cable
#define M  (0.3)  //Mass of object
#define I  (M*(a*a+L*L))/12 //inertial

//#define weight 0.45    //weight of object


Eigen::MatrixXd forceest::dynamics(Eigen::MatrixXd sigma_state){
  // std::cout<< "-------------" <<std::endl;
//    std::cout<< "I  " << I<<std::endl;
//    std::cout<< "M  " << M<<std::endl;
//    std::cout<< "L  " << L<<std::endl;
//    std::cout<< "r  " << r<<std::endl;
//    std::cout<< "a  " << I<<std::endl;
    Eigen::MatrixXd predict_sigma_state(this->x_size,this->x_sigmavector_size);

  // alpha = (this->x[omegap]-this->last_omega_p)/dt;
    alpha = lpf->filter((this->x[omegap]-this->last_omega_p)/dt);
    std::cout<< "dt"<<dt <<std::endl;
   //alpha = (this->x[omegap]-this->last_omega_p)/dt;

    //std::cout<<"now"<<this->x[omegap] <<std::endl;
    //  std::cout<<"lastb"<<this->last_omega_p <<std::endl;
    for(int i=0;i<this->x_sigmavector_size;i++){

        double theta_p = sigma_state(thetap,i) ;
        double omega_p = sigma_state(omegap,i);
        double FFx = sigma_state(FF_x,i);
        double FFz = sigma_state(FF_z,i);
        double FLx = sigma_state(FL_x,i);
        double FLz = sigma_state(FL_z,i);
        double acx = sigma_state(ac_x,i);
        double acz = sigma_state(ac_z,i);
        double apx = sigma_state(ap_x,i);
        double apz = sigma_state(ap_z,i);
        double theta_d = atan2(FLz , FLx);
        double theta_c = atan2(FFz , FFx);
        double FF_net = sqrt(FFx*FFx +FFz*FFz);
        double FL_net = sqrt(FLx*FLx +FLz*FLz);

        double alpha_p = (r/I)*(FF_net*1-FL_net*1);

//        std::cout << "-----------" <<std::endl;
//        std::cout << "alphap"<<alpha_p<<std::endl;
        //+alpha_p*r*cos(theta_p)
        //+alpha_p*r*sin(theta_p)
       // double alpha_p = (r/I)*(FF_net-FL_net*sin(theta_c+theta_d));

       // std::cout << "alpha_p" <<  alpha_p <<std::endl;


        predict_sigma_state(thetap,i) = theta_p+omega_p*dt;
        predict_sigma_state(omegap,i) =  omega_p;

        predict_sigma_state(ac_x,i) =  acx;
        predict_sigma_state(ac_z,i) =  acz;

        predict_sigma_state(FF_x,i) = FFx ;
        predict_sigma_state(FF_z,i) = FFz ;



        predict_sigma_state(ap_x,i) = acx - alpha*r*sin(theta_p) - omega_p*omega_p*r*cos(theta_p);
        predict_sigma_state(ap_z,i) = acz - alpha*r*cos(theta_p) + omega_p*omega_p*r*sin(theta_p);

        predict_sigma_state(FL_x,i) = M*apx - FFx;
        predict_sigma_state(FL_z,i) = M*apz + M*g - FFz;

    }
    return predict_sigma_state;


}
