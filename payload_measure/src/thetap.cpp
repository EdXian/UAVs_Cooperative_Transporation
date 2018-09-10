#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Imu.h"
#include "forceest.h"
#include "geometry_msgs/PoseStamped.h"
#include <tf/transform_datatypes.h>
#include "UKF/output.h"
#include "geometry_msgs/Point.h"
#include "kalmanfilter.h"

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"
#include "lpf.h"
#include "geometry_msgs/TwistStamped.h"

sensor_msgs::Imu imu_data;
Eigen::Matrix3d Q;


geometry_msgs::PoseStamped mocap_pose;
Eigen::Vector3d a_I , a_B;
UKF::output  ukf_data;
UKF::output  ukf2_data;
double fx ,fy ,fz;
double mocap_roll , mocap_yaw , mocap_pitch;
double theta_p;
double omega_p;
bool flag = true;
double yaw_bias ;
double imu_roll , imu_pitch , imu_yaw;
double x_bias , y_bias , z_bias;
double a_x_I,a_y_I,a_z_I;
double a_x_B,a_y_B,a_z_B;
bool imu_flag=true;
double w,x,y,z;

geometry_msgs::Point leader_force;

Eigen::Vector3d  omega_obj;
Eigen::Matrix3d R_B_I ; //body frame to inertial frame matrix

void imu_cb(const sensor_msgs::Imu::ConstPtr& msg){

  a_B.setZero(3);
  imu_data = *msg;
//  a_B<< -1*imu_data.linear_acceleration.x ,
//       -1*imu_data.linear_acceleration.y ,
//       -1*imu_data.linear_acceleration.z ;

  a_B<< imu_data.linear_acceleration.x ,
        imu_data.linear_acceleration.y ,
        imu_data.linear_acceleration.z ;

  omega_p = imu_data.angular_velocity.y;
  tf::Quaternion quat1(imu_data.orientation.x,imu_data.orientation.y, imu_data.orientation.z, imu_data.orientation.w);
  tf::Matrix3x3(quat1).getRPY(imu_roll, imu_pitch,  imu_yaw);

  theta_p = imu_pitch;

  //correct yaw angle bias in body frame
  if(flag){
    yaw_bias = imu_yaw;

    x_bias = imu_data.linear_acceleration.x;
    y_bias = imu_data.linear_acceleration.y;
    z_bias = imu_data.linear_acceleration.z;


    flag = false;
  }


  if( (imu_yaw - yaw_bias) < 0  ){

       imu_yaw = imu_yaw - yaw_bias +2*3.14159 ;
  }else{

    imu_yaw  = imu_yaw- yaw_bias;
  }


}
sensor_msgs::Imu object;
Eigen::Vector3d  a_obj;
double a_obj_x =0;
double a_obj_y = 0;
double a_obj_z = 0;
double obj_pitch , obj_roll,obj_yaw;

void imu2_cb(const sensor_msgs::Imu::ConstPtr& msg){

//  tfScalar x , y, z ,w;
//  tf::Quaternion quat(x , y , z , w);
//  tfScalar angle =  quat.getAngle();

    object = *msg;
    Eigen::Vector3d data;
    data<< object.linear_acceleration.x,
          object.linear_acceleration.y,
          object.linear_acceleration.z;

//  std::cout<<"========= imu2 =========" <<std::endl;
//  std::cout<<data<<std::endl;


 tf::Quaternion quat1(object.orientation.x,object.orientation.y, object.orientation.z, object.orientation.w);
 tf::Matrix3x3(quat1).getRPY(obj_roll, obj_pitch,  obj_yaw);
  omega_obj[0] = object.angular_velocity.x;
  omega_obj[1] = object.angular_velocity.y;
  omega_obj[2] = object.angular_velocity.z;

}


void ukf_cb(const UKF::output::ConstPtr& msg){
  ukf_data = *msg;
}

void ukf2_cb(const UKF::output::ConstPtr& msg){
  ukf2_data = *msg;
}



geometry_msgs::Point mocap_point;
void mocap_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
   mocap_pose= *msg;
fx = mocap_pose.pose.position.x;
fy = mocap_pose.pose.position.y;
fz = mocap_pose.pose.position.z;


mocap_point.x = fx;
mocap_point.y = fy;
mocap_point.z = fz;


x = mocap_pose.pose.orientation.x;
y = mocap_pose.pose.orientation.y;
z = mocap_pose.pose.orientation.z;
w = mocap_pose.pose.orientation.w;

tf::Quaternion quat1(mocap_pose.pose.orientation.x,mocap_pose.pose.orientation.y, mocap_pose.pose.orientation.z, mocap_pose.pose.orientation.w);
tf::Matrix3x3(quat1).getRPY(mocap_roll, mocap_pitch,  mocap_yaw);

if(mocap_yaw <0){
  mocap_yaw += 2*3.14159;
}

Q<< w*w+x*x-y*y-z*z  , 2*x*y-2*w*z ,            2*x*z+2*w*y,
    2*x*y +2*w*z           , w*w-x*x+y*y-z*z    ,2*y*z-2*w*x,
    2*x*z -2*w*y          , 2*y*z+2*w*x        ,w*w-x*x-y*y+z*z;


a_I = Q*a_B;
a_x_I = a_I[0];
a_y_I = a_I[1];
a_z_I = a_I[2] - 9.81;
Eigen::Vector3d data;
data<< object.linear_acceleration.x,
                   object.linear_acceleration.y,
                   object.linear_acceleration.z;
a_obj = Q*data;
std::cout<<"=================="<<std::endl;
std::cout<< data<<std::endl;
a_obj_x = a_obj[0] ;
 a_obj_y = a_obj[1];
 a_obj_z = a_obj[2] - 9.81;

}

geometry_msgs::TwistStamped vel, vel2;
void vel_cb(const geometry_msgs::TwistStamped::ConstPtr& msg){
 vel = *msg;
}
void vel2_cb(const geometry_msgs::TwistStamped::ConstPtr& msg){
vel2 = *msg;
}




int main(int argc, char **argv)
{
  ros::init(argc, argv, "thetap");
  ros::NodeHandle nh;

  ros::Subscriber imu_sub = nh.subscribe("/imu1/mavros/imu/data" , 2, imu_cb);
  ros::Subscriber imu2_sub = nh.subscribe("/imu2/mavros/imu/data" , 2, imu2_cb);
  ros::Subscriber mocap_sub = nh.subscribe("/vrpn_client_node/RigidBody1/pose" , 2 , mocap_cb);

  ros::Subscriber ukf2_sub = nh.subscribe("/leader_ukf/output",2,ukf2_cb);
  ros::Subscriber  ukf_sub = nh.subscribe("/follower_ukf/output",2,ukf_cb);
  ros::Publisher point_pub = nh.advertise<geometry_msgs::Point>("point5",10);
  ros::Publisher mtheta_p_pub = nh.advertise<geometry_msgs::Point>("theta_p4",10);
  ros::Publisher leader_force_pub = nh.advertise<geometry_msgs::Point>("leader_force3",2);

  ros::Subscriber vel_sub = nh.subscribe("/drone2/mavros/local_position/velocity",2,vel_cb);
  ros::Subscriber vel2_sub = nh.subscribe("/drone3/mavros/local_position/velocity",2,vel2_cb);


  geometry_msgs::Point theta;




  ros::Rate loop_rate(50);


  forceest forceest1(statesize,measurementsize);
  Eigen::MatrixXd mnoise;
  mnoise.setZero(measurementsize,measurementsize);
  mnoise   = 3e-3*Eigen::MatrixXd::Identity(measurementsize , measurementsize);

  mnoise(mthetap,mthetap) =  2e-3;
  mnoise(momegap,momegap) =  2e-3;
  mnoise(mac_x,mac_x) =  3e-3;
  mnoise(mac_z,mac_z) =  3e-3;
  mnoise(mFF_x,mFF_x) =  3e-3;
  mnoise(mFF_z,mFF_z) =  3e-3;


  std::cout<< "mnoise"<<std::endl;
  std::cout<<mnoise<<std::endl;

  forceest1.set_measurement_noise(mnoise);



  Eigen::MatrixXd pnoise;
  pnoise.setZero(statesize,statesize);
  //pnoise   = 5e-2*Eigen::MatrixXd::Identity(statesize , statesize);

  pnoise(thetap , thetap) = 1e-3;
  pnoise(omegap , omegap) = 1e-3;
  pnoise(ac_x , ac_x) = 4e-2;
  pnoise(ac_z , ac_z) = 4e-2;
  pnoise(ap_x , ap_x) = 4e-2;
  pnoise(ap_z , ap_z) = 4e-2;

  pnoise(FF_x , FF_x) = 3e-3;
  pnoise(FF_z , FF_z) = 3e-3;
  pnoise(FL_x , FL_x) = 1e-3;
  pnoise(FL_z , FL_z) = 1e-3;

  std::cout<< "pnoise"<<std::endl;
  std::cout<<pnoise<<std::endl;

  forceest1.set_process_noise(pnoise);


  Eigen::MatrixXd measurement_matrix;
  measurement_matrix.setZero(measurementsize,statesize);
  measurement_matrix(mthetap , mthetap) =1;
  measurement_matrix(momegap , momegap) =1;
  measurement_matrix(mac_x , mac_x) =1;
  measurement_matrix(mac_z , mac_z) =1;
  measurement_matrix(4 , 6) =1;
  measurement_matrix(5 , 7) =1;



  forceest1.set_measurement_matrix(measurement_matrix);
  std::cout <<"measurement_matrix"<<std::endl<<measurement_matrix<<std::endl;


  kalmanfilter kf(1,1);
  Eigen::MatrixXf A;
  Eigen::MatrixXf H;
  Eigen::MatrixXf Q;
  Eigen::MatrixXf R;
  Eigen::MatrixXf P;
  Eigen::VectorXf v;

  v.setZero(1);
  A.setZero(1,1);
  H.setZero(1,1);
  Q.setZero(1,1);
  R.setZero(1,1);
  P.setZero(1,1);
  A<<1;
  H<<1;
  Q<<7e-4;
  R<<1e-2;
  P<<1e-4;
  kf.setFixed(A,H,Q,R);
  v<<0;
  kf.setInitial(v , P);



  lpf lpf1( 5,0.02);


  double last_omegap_;
  double last_omegap;
  bool init =true;

  double time =ros::Time::now().toSec();
  double lastime =time;

  while(ros::ok()){

    time =ros::Time::now().toSec();
    forceest1.dt = time - lastime;
    lastime = time;


    forceest1.predict();
    Eigen::VectorXd measure;

    measure.setZero(6);

    measure << theta_p , omega_p ,a_x_I,a_z_I ,-ukf_data.force.x,-ukf_data.force.z ;

    if(init ){
     last_omegap_ = omega_p;
     init = false;
    }else{
      last_omegap_=  forceest1.x[omegap];
    }
     forceest1.last_omega_p = forceest1.x[omegap];
    forceest1.correct(measure);



    kf.predict();

    //double alpha = (forceest1.x[omegap] - last_omegap_)/ (forceest1.dt);
      double k = cos(0.5);
//    double r = 0.8;
//    double apx =a_x_I +alpha*r*sin(forceest1.x[thetap]) - forceest1.x[omegap]*forceest1.x[omegap]*r *cos(forceest1.x[thetap]);
//    double apz =a_z_I +alpha*r*cos(forceest1.x[thetap]) + forceest1.x[omegap]*forceest1.x[omegap]*r *sin(forceest1.x[thetap]);

    theta.x  = forceest1.x[ac_z];//forceest1.x[FF_z];
    theta.y  = a_obj_z; //object.angular_velocity.y;

    //theta.y  = ;//a_obj_x;// +0.4;//forceest1.x[FL_z];
    // a_obj_z;//
    //  theta.z = -(0.3* forceest1.x[ap_x]-  forceest1.x[FF_x]);
    // theta.z = -1*ukf2_data.force.z;
    //    theta.x  = a_x_I ;//forceest1.x[FF_z];
    //    theta.y  =  a_y_I   ;// +0.4;//forceest1.x[FL_z];
    //    theta.z  =  a_z_I;
    last_omegap = omega_p;

    leader_force.x = forceest1.x[FL_x];
    leader_force.z = forceest1.x[FL_z];

//    theta.x  = leader_force.x;//forceest1.x[FF_z];
//    theta.y  = leader_force.z; //object.angular_velocity.y;



    leader_force_pub.publish(leader_force);
    mtheta_p_pub.publish(theta);
    point_pub.publish(mocap_point);



 std::cout<<"---------"<<std::endl<< forceest1.x <<std::endl;
//    std::cout << atan2(ukf_data.force.z ,ukf_data.force.x )* 57<<std::endl;

    ros::spinOnce();
   loop_rate.sleep();
  }


  return 0;
}
