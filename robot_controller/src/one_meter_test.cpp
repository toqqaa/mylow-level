#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

geometry_msgs::Twist vel;
ros::Publisher pub;


void sleep_timer(float time) {
  float i = 0;

  while (i < time) {
    pub.publish(vel);
    ROS_INFO("count the duration %f ", i);
    sleep(1);
    i++;
  }
}


int main(int argc, char** argv) {

    ros::init(argc, argv, "test_robot_node");
    ros::NodeHandle nh;
    
     pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
    ros::Rate loop_rate(2);
    
     vel.linear.x = 0.25;
    vel.angular.z = 0.0;
    sleep_timer(4);
    vel.linear.x = 0.0;
    vel.angular.z = 0.0;
    pub.publish(vel);
    while (ros::ok())
    {
        // pub.publish(vel);
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}

