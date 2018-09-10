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
#define pi 3.1415926
float KPx = 0.4;
float KPy = 0.4;
float KPz = 0.4;

using namespace std;
typedef struct vir
{
    float roll;
    float x;
    float y;
    float z;
};
typedef struct displacement
{
    float x;
    float y;
};


mavros_msgs::State current_state, current_state2, current_state3, current_state4;
void state_cb(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}
void state_cb2(const mavros_msgs::State::ConstPtr& msg) {
    current_state2 = *msg;
}
void state_cb3(const mavros_msgs::State::ConstPtr& msg) {
    current_state3 = *msg;
}


geometry_msgs::PoseStamped host_mocap, host_mocap2, host_mocap3, host_mocap4;
void host_pos(const geometry_msgs::PoseStamped::ConstPtr& msg)
{

	host_mocap = *msg;

}
void host_pos2(const geometry_msgs::PoseStamped::ConstPtr& msg)
{

	host_mocap2 = *msg;

}
void host_pos3(const geometry_msgs::PoseStamped::ConstPtr& msg)
{

	host_mocap3 = *msg;

}


void follow(vir& vir, geometry_msgs::PoseStamped& host_mocap, geometry_msgs::TwistStamped* vs, displacement &dis_host, geometry_msgs::PoseStamped& nbr_mocap, displacement &dis_nbr, geometry_msgs::PoseStamped& nbr2_mocap, displacement &dis_nbr2)
{
float errx, erry, errz, err_roll;
float ux, uy, uz;
float local_x, local_y;
float local_x1, local_y1;
float local_x2, local_y2;
float dis_x1, dis_y1;
float dis_x2, dis_y2;

local_x = cos(vir.roll)*dis_host.x+sin(vir.roll)*dis_host.y;
local_y = -sin(vir.roll)*dis_host.x+cos(vir.roll)*dis_host.y;

dis_x1 = dis_host.x - dis_nbr.x;
dis_y1 = dis_host.y - dis_nbr.y;

local_x1 = cos(vir.roll)*dis_x1+sin(vir.roll)*dis_y1;
local_y1 = -sin(vir.roll)*dis_x1+cos(vir.roll)*dis_y1;

dis_x2 = dis_host.x - dis_nbr2.x;
dis_y2 = dis_host.y - dis_nbr2.y;

local_x2 = cos(vir.roll)*dis_x2+sin(vir.roll)*dis_y2;
local_y2 = -sin(vir.roll)*dis_x2+cos(vir.roll)*dis_y2;

errx = 1.3*(vir.x - host_mocap.pose.position.x + local_x) + (nbr_mocap.pose.position.x - host_mocap.pose.position.x + local_x1) + (nbr2_mocap.pose.position.x - host_mocap.pose.position.x + local_x2);
erry = 1.3*(vir.y - host_mocap.pose.position.y + local_y) + (nbr_mocap.pose.position.y - host_mocap.pose.position.y + local_y1) + (nbr2_mocap.pose.position.y - host_mocap.pose.position.y + local_y2);
errz = 1.3*(vir.z - host_mocap.pose.position.z + 0) + (nbr_mocap.pose.position.z - host_mocap.pose.position.z + 0) + (nbr2_mocap.pose.position.z - host_mocap.pose.position.z + 0);

ux = KPx*errx;
uy = KPy*erry;
uz = KPz*errz;

vs->twist.linear.x = ux;
vs->twist.linear.y = uy;
vs->twist.linear.z = uz;

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
    ros::init(argc, argv, "formation_three");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
                                ("drone1/mavros/state", 10, state_cb);
    ros::Subscriber state_sub2 = nh.subscribe<mavros_msgs::State>
                                ("drone2/mavros/state", 10, state_cb2);
	ros::Subscriber state_sub3 = nh.subscribe<mavros_msgs::State>
                                ("drone3/mavros/state", 10, state_cb3);
    //ros::Publisher mocap_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
    //                               ("drone1/mavros/mocap/pose", 1);
    //ros::Publisher mocap_pos_pub2 = nh.advertise<geometry_msgs::PoseStamped>
    //                               ("drone2/mavros/mocap/pose", 1);
  //ros::Publisher mocap_pos_pub3 = nh.advertise<geometry_msgs::PoseStamped>
    //                               ("drone3/mavros/mocap/pose", 1);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
                                       ("drone1/mavros/cmd/arming");
    ros::ServiceClient arming_client2 = nh.serviceClient<mavros_msgs::CommandBool>
                                       ("drone2/mavros/cmd/arming");
	ros::ServiceClient arming_client3 = nh.serviceClient<mavros_msgs::CommandBool>
                                       ("drone3/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
                                         ("drone1/mavros/set_mode");
    ros::ServiceClient set_mode_client2 = nh.serviceClient<mavros_msgs::SetMode>
                                         ("drone2/mavros/set_mode");
	ros::ServiceClient set_mode_client3 = nh.serviceClient<mavros_msgs::SetMode>
                                         ("drone3/mavros/set_mode");
    ros::Subscriber host_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/RigidBody1/pose", 1, host_pos);

    ros::Subscriber host_sub2 = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/RigidBody2/pose", 1, host_pos2);
	
        ros::Subscriber host_sub3 = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/RigidBody3/pose", 1, host_pos3);

    ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::TwistStamped>("drone1/mavros/setpoint_velocity/cmd_vel", 1);

    ros::Publisher local_vel_pub2 = nh.advertise<geometry_msgs::TwistStamped>("drone2/mavros/setpoint_velocity/cmd_vel", 1);
	
        ros::Publisher local_vel_pub3 = nh.advertise<geometry_msgs::TwistStamped>("drone3/mavros/setpoint_velocity/cmd_vel", 1);

 
    // The setpoint publishing rate MUST be faster than 2Hz.
	// Using multi thread
	ros::AsyncSpinner spinner(6);  
 	spinner.start();
    ros::Rate rate(50);

    // Wait for FCU connection.
    while (ros::ok() && current_state.connected && current_state2.connected && current_state3.connected) {
        /*mocap_pos_pub.publish(host_mocap);
	mocap_pos_pub2.publish(host_mocap2);
        mocap_pos_pub3.publish(host_mocap3);*/
        ros::spinOnce();
        rate.sleep();
    }

    
    geometry_msgs::TwistStamped vs, vs2, vs3;
	vir vir1;
        displacement dis1, dis2, dis3;

        dis1.x = -0.2;
	dis1.y = -0.9;
        dis2.x= -0.2;
	dis2.y = 0.9;
	dis3.x= 0.7;
	dis3.y = 0;

	vir1.x = 0;
	vir1.y = 0;
	vir1.z = 0.5;
	vir1.roll = 0;

    vs.twist.linear.x = 0;
    vs.twist.linear.y = 0;
    vs.twist.linear.z = 0;
    vs.twist.angular.x = 0;
    vs.twist.angular.y = 0;
    vs.twist.angular.z = 0;

    vs2.twist.linear.x = 0;
    vs2.twist.linear.y = 0;
    vs2.twist.linear.z = 0;
    vs2.twist.angular.x = 0;
    vs2.twist.angular.y = 0;
    vs2.twist.angular.z = 0;

	vs3.twist.linear.x = 0;
    vs3.twist.linear.y = 0;
    vs3.twist.linear.z = 0;
    vs3.twist.angular.x = 0;
    vs3.twist.angular.y = 0;
    vs3.twist.angular.z = 0;


    //send a few setpoints before starting
   for(int i = 100; ros::ok() && i > 0; --i){
    local_vel_pub.publish(vs);
	local_vel_pub2.publish(vs2);
	local_vel_pub3.publish(vs3);
        //mocap_pos_pub.publish(host_mocap);
  //mocap_pos_pub2.publish(host_mocap2);
    //    mocap_pos_pub3.publish(host_mocap3);
	ros::spinOnce();
    rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    //ros::Time last_request2 = ros::Time::now();
	//ros::Time last_request3 = ros::Time::now();

    while (ros::ok()) {

        //mocap_pos_pub.publish(host_mocap);
  //mocap_pos_pub2.publish(host_mocap2);
        //mocap_pos_pub3.publish(host_mocap3);
	//mocap_pos_pub4.publish(host_mocap4);

        if (current_state.mode != "OFFBOARD" &&
                (ros::Time::now() - last_request > ros::Duration(5.0))) {
            if( set_mode_client.call(offb_set_mode) && set_mode_client2.call(offb_set_mode) && set_mode_client3.call(offb_set_mode) &&
                    offb_set_mode.response.mode_sent) {
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {

            if (!current_state.armed &&
                    (ros::Time::now() - last_request > ros::Duration(5.0))) {
                if( arming_client.call(arm_cmd) && arming_client2.call(arm_cmd) && arming_client3.call(arm_cmd) &&
                        arm_cmd.response.success) {
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

	/*if (current_state2.mode != "OFFBOARD" &&
                (ros::Time::now() - last_request2 > ros::Duration(5.0))) {
            if( set_mode_client2.call(offb_set_mode) &&
                    offb_set_mode.response.success) {
                ROS_INFO("Offboard2 enabled");
            }
            last_request2 = ros::Time::now();
        } else {

            if (!current_state2.armed &&
                    (ros::Time::now() - last_request2 > ros::Duration(5.0))) {
                if( arming_client2.call(arm_cmd) &&
                        arm_cmd.response.success) {
                    ROS_INFO("Vehicle2 armed");
                }
                last_request2 = ros::Time::now();
            }
        }

	if (current_state3.mode != "OFFBOARD" &&
                (ros::Time::now() - last_request3 > ros::Duration(5.0))) {
            if( set_mode_client3.call(offb_set_mode) &&
                    offb_set_mode.response.success) {
                ROS_INFO("Offboard3 enabled");
            }
            last_request3 = ros::Time::now();
        } else {

            if (!current_state3.armed &&
                    (ros::Time::now() - last_request3 > ros::Duration(5.0))) {
                if( arming_client3.call(arm_cmd) &&
                        arm_cmd.response.success) {
                    ROS_INFO("Vehicle3 armed");
                }
                last_request3 = ros::Time::now();
            }
        }*/


        int c = getch();
	//ROS_INFO("C: %d",c);
        if (c != EOF) {
            switch (c) {
            case 65:    // key up
                vir1.z += 0.1;
                break;
            case 66:    // key down
                vir1.z += -0.1;
                break;
            case 67:    // key CW
                vir1.roll -= 0.1;
                break;
            case 68:    // key CCW
                vir1.roll += 0.1;
                break;
			case 119:    // key foward
                vir1.x += 0.1;
                break;
            case 120:    // key back
                vir1.x += -0.1;
                break;
            case 97:    // key left
                vir1.y += 0.1;
                break;
            case 100:    // key right
                vir1.y -= 0.1;
                break;
	    	case 115:    // key right
			{
			vir1.x = 0;
        		vir1.y = 0;
			vir1.z = 0;
			vir1.roll = 0;
                break;
			}
			case 108:    // close arming
			{
			offb_set_mode.request.custom_mode = "MANUAL";
			set_mode_client.call(offb_set_mode);
			set_mode_client2.call(offb_set_mode);
			set_mode_client3.call(offb_set_mode);

			arm_cmd.request.value = false;
			arming_client.call(arm_cmd);
			arming_client2.call(arm_cmd);
			arming_client3.call(arm_cmd);

            break;
			}
            case 63:
                return 0;
                break;
            }
        }

        ROS_INFO("setpoint: %.2f, %.2f, %.2f, %.2f", vir1.x, vir1.y, vir1.z,vir1.roll/pi*180);
        ros::spinOnce();
        follow(vir1,host_mocap,&vs,dis1,host_mocap2,dis2,host_mocap3,dis3);        //be careful of the sign
        //mocap_pos_pub.publish(host_mocap);
        local_vel_pub.publish(vs);
        ros::spinOnce();
        follow(vir1,host_mocap2,&vs2,dis2,host_mocap,dis1,host_mocap3,dis3);     //be careful of the sign
        //mocap_pos_pub2.publish(host_mocap2);
        local_vel_pub2.publish(vs2);
        ros::spinOnce();
        follow(vir1,host_mocap3,&vs3,dis3,host_mocap,dis1,host_mocap2,dis2);     //be careful of the sign
        //mocap_pos_pub3.publish(host_mocap3);
        local_vel_pub3.publish(vs3);
        //follow(vir1,host_mocap4,&vs4,0.9,0);     //be careful of the sign

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

