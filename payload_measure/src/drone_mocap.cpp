#include <ros/ros.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Point.h"
class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {
    pub_vel= n_.advertise<geometry_msgs::Point>("/connector/velocity",2);
    pub_ = n_.advertise<geometry_msgs::PoseStamped>("/mavros/mocap/pose", 2);
    sub_ = n_.subscribe("/vrpn_client_node/RigidBody1/pose", 2, &SubscribeAndPublish::callback, this);
    sub_vel = n_.subscribe("/mavros/local_position/velocity",2 ,&SubscribeAndPublish::vel_callback, this);
  }
  void vel_callback(const geometry_msgs::TwistStamped::ConstPtr& msg){
    point.x = msg->twist.linear.x;
    point.y =msg->twist.linear.y;
    point.z = msg->twist.linear.z;
    pub_vel.publish(point);
  }
  void callback(const geometry_msgs::PoseStamped::ConstPtr& input)
  {
    pub_.publish(input);
  }

private:
  geometry_msgs::Point point;
  ros::NodeHandle n_;
  ros::Publisher pub_;
  ros::Publisher pub_vel;
  ros::Subscriber sub_;
  ros::Subscriber sub_vel;
};//End of class SubscribeAndPublish

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "subscribe_and_publish");

  //Create an object of class SubscribeAndPublish that will take care of everything
  SubscribeAndPublish SAPObject;

  ros::spin();

  return 0;
}
