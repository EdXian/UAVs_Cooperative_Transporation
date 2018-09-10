#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#define interval
double upper_bound = 1.5;
double lower_bound = -1.5;
double zero = 1.0;
int current_state=0;
int last_state=0;
int controlloer_state;
int score;

geometry_msgs::Point posx;
double en_time_threshold = -1;
double dis_time_threshold =-1;
enum zone{
    positive_zone = 5,
    zero_zone = 0 ,
    negative_zone = -5
};


enum control_state{
  positive_engaged = 5,
  disengaged = 0,
  negative_engaged = -5
};
geometry_msgs::Point leader_force ;

void force_cb(const geometry_msgs::Point::ConstPtr& msg){
leader_force  = *msg;

}

double last_pose_x;
geometry_msgs::TwistStamped vel;
geometry_msgs::PoseStamped pose;
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr & msg)
{
  pose = *msg;
  if(abs( pose.pose.position.x - last_pose_x)>0.3){
    posx.x = last_pose_x;
  }else{
      posx.x = pose.pose.position.x;
  }

  //posx.x = pose.twist.linear.z;
  last_pose_x = pose.pose.position.x;
}
geometry_msgs::TwistStamped vel2;
geometry_msgs::PoseStamped pose2;
void pose2_cb(const geometry_msgs::PoseStamped::ConstPtr & msg)
{
  pose2 = *msg;
  posx.y = pose2.pose.position.x;
 //posx.y = pose2.twist.linear.z;
}

struct  box
{
  int a =5;
  float b=0.3;
};
struct box *pxxx(void){
  //float a= 0.534;
  //float *p=&a;
  struct box box1;
  box1.a = 2;
  box1.b = 0.034;
  box *pbox = &box1;
  return pbox;
}

geometry_msgs::Point output;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "trigger");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("/trigger", 3,force_cb);
  ros::Subscriber pose_sub = nh.subscribe("/drone2/mavros/local_position/pose", 3,pose_cb);
  ros::Subscriber pose2_sub = nh.subscribe("/drone3/mavros/local_position/pose", 3,pose2_cb);
  ros::Publisher pub_pose = nh.advertise<geometry_msgs::Point>("posex",3);
  ros::Publisher pub = nh.advertise<geometry_msgs::Point>("controller_state",3);
  ros::Rate loop_rate(50);

   box *p  = pxxx();
   std::cout << " p = " <<p->a  << " " << p->b<<std::endl;

  double time =ros::Time::now().toSec();
  double lastime =time;
  double last_in_update_time = 0;
  double last_out_update_time = 0;
  std::list<double> force_vector;
  double force;
  double avg;

  while(ros::ok()){

    if(force_vector.size()>8){
      force_vector.pop_front();
    }

    force = 0;

    force_vector.push_back(leader_force.z);
    std::cout<< "============" << force_vector.size()<< "============" <<std::endl;
    for(std::list<double>::iterator it= force_vector.begin();it!=force_vector.end();it++){
      std::cout<<*(it)<<std::endl;
      force += *(it);
    }
    if(force_vector.size()>0){
       avg = force/(force_vector.size());
    }else{
      avg = 0;
    }



    last_state = current_state;
    if(avg > upper_bound){
      current_state = positive_zone;
    }else if(avg < lower_bound){
      current_state = negative_zone ;
    }else{
      current_state = zero_zone;
    }

#ifdef interval

    time = ros::Time::now().toSec();



    if((current_state == positive_zone) && (last_state ==  zero_zone) ){

      last_in_update_time = time;

    }
    if((controlloer_state == positive_zone) && (leader_force.z<zero) ){

      last_out_update_time = time;
    }
    if((current_state == negative_zone) && (last_state ==  zero_zone)){

       last_in_update_time = time;
    }
    if((controlloer_state == negative_zone) && (leader_force.z>-1*zero)){

     last_out_update_time = time;
    }

    double dt_in = time - last_in_update_time;
    double dt_out = time - last_out_update_time;

    if((current_state == positive_zone) && (dt_in>en_time_threshold)  ){
      controlloer_state = positive_engaged;
      //last_in_update_time = time;

    }
    if((controlloer_state == positive_zone) && (leader_force.z<zero) && (dt_out>dis_time_threshold )){
      controlloer_state = disengaged;
      //last_out_update_time = time;
    }
    if((current_state == negative_zone)  && (dt_in>en_time_threshold )){
      controlloer_state = negative_engaged;
       //last_in_update_time = time;
    }
    if((controlloer_state == negative_zone) && (leader_force.z>-1*zero)&& (dt_out>dis_time_threshold )){
      controlloer_state = disengaged;
     //last_out_update_time = time;
    }

#else
    lastime = time;
   time = ros::Time::now().toSec();
   double dt = time - last_update_time;
   if((leader_force.z>2.5) && (last_state ==  zero_zone) &&(dt>0.3)  ){
     controlloer_state = positive_engaged;
     last_update_time = time;

   }
   if((controlloer_state == positive_engaged) && (leader_force.z<1.1) && (dt>0.3) ){
     controlloer_state = disengaged;
     last_update_time = time;
   }
   if((leader_force.z<-2.5) && (last_state ==  zero_zone)&&(dt>0.3)){
     controlloer_state = negative_engaged;
     last_update_time = time;
   }
   if((controlloer_state == negative_engaged) && (leader_force.z>-1.1)&&(dt>0.3)){
     controlloer_state = disengaged;
     last_update_time = time;
   }
#endif

    output.x = avg;
    output.y = leader_force.z;
    output.z = controlloer_state;
    posx.z = posx.x - posx.y;
    pub.publish(output);
    pub_pose.publish(posx);

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
