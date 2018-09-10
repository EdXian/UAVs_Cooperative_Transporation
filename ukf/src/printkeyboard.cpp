#include <ros/ros.h>
#include <cstdio>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
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
int main(int argc, char **argv)
{
  ros::init(argc, argv, "printkeyboard");
  ros::NodeHandle nh;
  ros::Rate rate(1);
  while(ros::ok()){
  int c = getch();
  ROS_INFO("C: %d",c);
  ros::spinOnce();
  rate.sleep();
  }

}
