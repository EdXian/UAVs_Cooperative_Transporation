/**
 * @file offb_main.cpp
 * @author Julian Oes <julian@oes.ch>
 * @license BSD 3-clause
 *
 * @brief ROS node to do offboard control of PX4 through MAVROS.
 *
 * Initial code taken from http://dev.px4.io/ros-mavros-offboard.html
 */


#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <cstdio>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <geometry_msgs/Point.h>
#define pi 3.1415926

int flag=0;
float KPx = 1.5;
float KPy = 1.5;
float KPz = 1.5;
float KProll = 1;
bool force_control;
int flag1 = 0;
using namespace std;
typedef struct vir
{
    float roll;
    float x;
    float y;
    float z;
};

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}
geometry_msgs::Point leader_force;
void leader_force_cb(const geometry_msgs::Point::ConstPtr& msg) {
    leader_force = *msg;
}
geometry_msgs::PoseStamped host_mocap;
void host_pos(const geometry_msgs::PoseStamped::ConstPtr& msg)
{

  host_mocap = *msg;

}
float qua2eul(geometry_msgs::PoseStamped& host_mocap)
{
    float pitch,yaw,roll,qx2,qy2,qz2,qw2;
    qx2=(host_mocap.pose.orientation.x)*(host_mocap.pose.orientation.x);
    qy2=(host_mocap.pose.orientation.y)*(host_mocap.pose.orientation.y);
    qz2=(host_mocap.pose.orientation.z)*(host_mocap.pose.orientation.z);
    qw2=(host_mocap.pose.orientation.w)*(host_mocap.pose.orientation.w);
    roll = atan2(2*host_mocap.pose.orientation.z*host_mocap.pose.orientation.w+2*host_mocap.pose.orientation.x*host_mocap.pose.orientation.y , 1 - 2*qy2 - 2*qz2);
    //roll = asin(2*host_mocap.pose.orientation.x*host_mocap.pose.orientation.y + 2*host_mocap.pose.orientation.z*host_mocap.pose.orientation.w);
    //pitch = atan2(2*host_mocap.pose.orientation.x*host_mocap.pose.orientation.w-2*host_mocap.pose.orientation.y*host_mocap.pose.orientation.z , 1 - 2*qx2 - 2*qz2);
  //ROS_INFO("eul: %.3f, %.3f, %.3f", pitch/pi*180, yaw/pi*180, roll/pi*180);

    return roll;
}
void follow(vir& vir, geometry_msgs::PoseStamped& host_mocap, geometry_msgs::TwistStamped* vs, float dis_x, float dis_y)
{
float errx, erry, errz, err_roll;
float ux, uy, uz, uroll;
//float dis_x = 0, dis_y = -0.5;
float local_x, local_y;
double FL_x, FL_y, FL_z;
local_x = cos(vir.roll)*dis_x+sin(vir.roll)*dis_y;
local_y = -sin(vir.roll)*dis_x+cos(vir.roll)*dis_y;

errx = vir.x - host_mocap.pose.position.x - local_x;
erry = vir.y - host_mocap.pose.position.y - local_y;
errz = vir.z - host_mocap.pose.position.z - 0;
err_roll = vir.roll - qua2eul(host_mocap);
if(err_roll>pi)
err_roll = err_roll - 2*pi;
else if(err_roll<-pi)
err_roll = err_roll + 2*pi;
FL_x = leader_force.x;
//ROS_INFO("err_roll: %.3f",err_roll);
//ROS_INFO("FL_x = %f", FL_x);
if(force_control){
  ux = KPy*errx;
  uy = KPy*erry;
  uz = KPz*errz;
  if(FL_x > 3.5){
    ux = 0.08 * FL_x;
      if(ux > 0.5){
        ux = 0.5;
                  }
    flag1 = 1;
                }
  if(FL_x < 3.5 && flag1 == 1){
    vir.x = host_mocap.pose.position.x;
    flag1 = 0;
  }
  if(FL_x < -3.5){
    ux = 0.08 * FL_x;
      if(ux < -0.5){
        ux = -0.5;
                  }
    flag1 = 1;
  }
  if(FL_x > -3.5 && flag1 == 1){
    vir.x = host_mocap.pose.position.x;
    flag1 = 0;
  }
}
ux = KPx*errx;
uy = KPy*erry;
uz = KPz*errz;
uroll = KProll*err_roll;

vs->twist.linear.x = ux;
vs->twist.linear.y = uy;
vs->twist.linear.z = uz;
vs->twist.angular.z = uroll;

}
/*
 * Taken from
 * http://stackoverflow.com/questions/421860/capture-characters-from-standard-input-without-waiting-for-enter-to-be-pressed
 *
 * @return the character pressed.
 */
char getch()
{
    int flags = fcntl(0, F_GETFL, 0);
    fcntl(0, F_SETFL, flags | O_NONBLOCK);

    char buf = 0;
    struct termios old = {0};
    if (tcgetattr(0, &old) < 0) {
        perror("tcsetattr()");
    }
    old.c_lflag &= ~ICANON;
    old.c_lflag &= ~ECHO;
    old.c_cc[VMIN] = 1;
    old.c_cc[VTIME] = 0;
    if (tcsetattr(0, TCSANOW, &old) < 0) {
        perror("tcsetattr ICANON");
    }
    if (read(0, &buf, 1) < 0) {
        //perror ("read()");
    }
    old.c_lflag |= ICANON;
    old.c_lflag |= ECHO;
    if (tcsetattr(0, TCSADRAIN, &old) < 0) {
        perror ("tcsetattr ~ICANON");
    }
    return (buf);
}

/*
 * Call main using `rosrun offb offb_main`.
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "follow_test");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
                                ("drone1/mavros/state", 2, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
                                   ("drone1/mavros/setpoint_position/local", 2);
    //ros::Publisher mocap_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
    //                               ("drone2/mavros/mocap/pose", 5);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
                                       ("drone1/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
                                         ("drone1/mavros/set_mode");
    ros::Subscriber host_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/RigidBody1/pose", 2, host_pos);

    ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::TwistStamped>("drone1/mavros/setpoint_velocity/cmd_vel", 2);
    ros::Subscriber leader_force_sub = nh.subscribe<geometry_msgs::Point>("/leader_force", 2, leader_force_cb);
    // The setpoint publishing rate MUST be faster than 2Hz.
    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::Rate rate(50);
    force_control = false;
    // Wait for FCU connection.
    while (ros::ok() && current_state.connected) {
  //mocap_pos_pub.publish(host_mocap);
        ros::spinOnce();
        rate.sleep();
    }


    geometry_msgs::PoseStamped pose;
    geometry_msgs::TwistStamped vs;
  vir vir2;

    vs.twist.linear.x = 0;
    vs.twist.linear.y = 0;
    vs.twist.linear.z = 0;
    vs.twist.angular.x = 0;
    vs.twist.angular.y = 0;
    vs.twist.angular.z = 0;

  vir2.x = -0.8;
  vir2.y = -0.5;
  vir2.z = 0.7;
  vir2.roll = 0;

    //send a few setpoints before starting
   for(int i = 100; ros::ok() && i > 0; --i){
        local_vel_pub.publish(vs);
    //mocap_pos_pub.publish(host_mocap);
    ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
  //ros::Time last_request(0);

    while (ros::ok()) {
  //mocap_pos_pub.publish(host_mocap);
        if (current_state.mode != "OFFBOARD" &&
                (ros::Time::now() - last_request > ros::Duration(5.0))) {
            if( set_mode_client.call(offb_set_mode) &&
                    offb_set_mode.response.mode_sent) {
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {

            if (!current_state.armed &&
                    (ros::Time::now() - last_request > ros::Duration(5.0))) {
                if( arming_client.call(arm_cmd) &&
                        arm_cmd.response.success) {
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }


  //take off

  /*if(host_mocap.pose.position.z>=0.5 && flag==0)
  {
    mocap_pos_pub.publish(host_mocap);
    vs.twist.linear.x = 0;
    vs.twist.linear.y = 0;
    vs.twist.linear.z = 0;
    vs.twist.angular.x = 0;
    vs.twist.angular.y = 0;
    vs.twist.angular.z = 0;
    flag = 1;
  }*/



        int c = getch();
  //ROS_INFO("C: %d",c);
        if (c != EOF) {
            switch (c) {
            case 56:    // key up,Num 8
                vir2.z += 0.05;
                break;
            case 50:    // key down,Num 2
                vir2.z += -0.05;
                break;
            case 67:    // key CW(->)
                //vir2.roll -= 0.05;
                break;
            case 68:    // key CCW(<-)
                //vir2.roll += 0.05;
                break;
            case 119:    // key foward
                vir2.x += 0.05;
                break;
            case 120:    // key back
                vir2.x += -0.05;
                break;
            case 97:    // key left
                vir2.y += 0.05;
                break;
            case 100:    // key right
                vir2.y -= 0.05;
                break;
            case 102:    // stop, key F
              {
                //vir2.x = -0.6;
                vir2.y = -0.5;
                vir2.z = 0;
                vir2.roll = 0;
                break;
               }
            case 107:    // close arming key K
               {
                offb_set_mode.request.custom_mode = "MANUAL";
                set_mode_client.call(offb_set_mode);
                arm_cmd.request.value = false;
                arming_client.call(arm_cmd);
                 break;
                }
            case 49: // change to force control, key "1"
                force_control = true;
                break;
            case 63:
                return 0;
                break;
            }
        }
          if(vir2.roll>pi)
          vir2.roll = vir2.roll - 2*pi;
          else if(vir2.roll<-pi)
          vir2.roll = vir2.roll + 2*pi;

          //ROS_INFO("setpoint: %.2f, %.2f, %.2f, %.2f", vir2.x, vir2.y, vir2.z, vir2.roll/pi*180);
          follow(vir2,host_mocap,&vs,0,-0.5);
          //mocap_pos_pub.publish(host_mocap);
          local_vel_pub.publish(vs);

        //ros::spinOnce();
          rate.sleep();
    }

    return 0;
}

