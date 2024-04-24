#include <ros/ros.h>
#include <robot_controller/McuData.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

double RobotPose_x,  RobotPose_y,
RobotPose_theta, RobotLinearVel,  RobotAngularVel;

void MCUCallback(const robot_controller::McuData::ConstPtr& msg)
{
  RobotPose_x=msg->RobotPose_x;
  RobotPose_y=msg->RobotPose_y;
  RobotPose_theta=msg->RobotPose_theta;
  RobotLinearVel=msg->RobotLinearVel;
  RobotAngularVel=msg->RobotAngularVel;
}

  int main(int argc, char** argv) {
    
    
    ros::init(argc, argv, "robot_Control");
    ros::NodeHandle nh;

    tf::TransformBroadcaster broadcaster;
    ros::Subscriber mcu_sub;
    nav_msgs::Odometry odom_msg;
    ros::Publisher odom_pub;
    // tf::TransformBroadcaster odom_broadcaster;
    
    mcu_sub = nh.subscribe("mcu_data", 50, MCUCallback);
    odom_pub = nh.advertise<nav_msgs::Odometry>("/noisy_odom", 50);

    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();
    ros::Rate loop_rate(50);

    while (ros::ok())
    {
    ros::spinOnce();// check for incoming messages    
    current_time = ros::Time::now();
    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(RobotPose_theta);
       
    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_footprint";

    odom_trans.transform.translation.x = RobotPose_x;
    odom_trans.transform.translation.y = RobotPose_y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    // odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS

    odom_msg.header.stamp = current_time;
    odom_msg.header.frame_id = "odom";

    //set the position
    odom_msg.pose.pose.position.x = RobotPose_x;
    odom_msg.pose.pose.position.y = RobotPose_y;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation = odom_quat;

    //set the velocity
    odom_msg.child_frame_id = "base_footprint";
    odom_msg.twist.twist.linear.x = RobotLinearVel;
    odom_msg.twist.twist.linear.y = 0;
    odom_msg.twist.twist.angular.z = RobotAngularVel;

        // odom_msg.twist.covariance={0.01 , 0.0,  0.0 , 0.0 , 0.0,  0.0,
        //                        0.0,  0.01 , 0.0,  0.0 , 0.0 , 0.0,
        //                        0.0 ,  0.0 ,0.01,  0.0,  0.0 , 0.0,
        //                        0.0 ,  0.0 , 0.0 , 0.1 , 0.0 , 0.0,
        //                        0.0 ,  0.0 , 0.0 , 0.0,  0.1 , 0.0,
        //                        0.0 ,  0.0 , 0.0  ,0.0 , 0.0 , 0.1};
                               
        // odom_msg.pose.covariance={0.01 , 0.0,  0.0 , 0.0 , 0.0,  0.0,
        //                        0.0,  0.01 , 0.0,  0.0 , 0.0 , 0.0,
        //                        0.0 ,  0.0 ,0.01,  0.0,  0.0 , 0.0,
        //                        0.0 ,  0.0 , 0.0 , 0.1 , 0.0 , 0.0,
        //                        0.0 ,  0.0 , 0.0 , 0.0,  0.1 , 0.0,
        //                        0.0 ,  0.0 , 0.0  ,0.0 , 0.0 , 0.1};  

    //publish the message
    odom_pub.publish(odom_msg);

     last_time = current_time; 
    loop_rate.sleep();
    }
    
    return 0;
}

