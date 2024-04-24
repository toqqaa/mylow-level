#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/Imu.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

ros::Publisher imu_pub;
ros::Subscriber imu_raw_sub;
sensor_msgs::Imu imu_data;
ros::Time current_time, last_time;

// Create a Quaternion
tf2::Quaternion quaternion;

double roll = 0.0;  // roll angle
double pitch = 0.0; // pitch angle
double yaw = 0.0;   // yaw angle

double linear_acc_x = 0.0;  // linear acceleration in x
double linear_acc_y = 0.0;  // linear acceleration in y
double linear_acc_z = 0.0;  // linear acceleration in z

double mag_x = 0.0;  // magnetometer data x
double mag_y = 0.0;  // magnetometer data y
double mag_z = 0.0;  // magnetometer data z

void IMUCallback(const std_msgs::Float32MultiArray::ConstPtr &raw_imu_msg)
{
    yaw = raw_imu_msg->data[0];
    pitch = raw_imu_msg->data[1];
    roll = raw_imu_msg->data[2];

    linear_acc_x = raw_imu_msg->data[3];
    linear_acc_y = raw_imu_msg->data[4];
    linear_acc_z = raw_imu_msg->data[5];

    mag_x = raw_imu_msg->data[6];
    mag_y = raw_imu_msg->data[7];
    mag_z = raw_imu_msg->data[8];
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "imu_publisher");
    ros::NodeHandle nh;

    imu_raw_sub = nh.subscribe("imu_raw", 50, IMUCallback);
    imu_pub = nh.advertise<sensor_msgs::Imu>("imu_data", 50);

    current_time = ros::Time::now();
    last_time = ros::Time::now();
    ros::Rate loop_rate(50);
    while (ros::ok())
    {
        ros::spinOnce();

        // Set the quaternion using Euler angles
        quaternion.setRPY(roll, pitch, yaw);

        // Update orientation in imu_data message
        imu_data.orientation.x = quaternion.x();
        imu_data.orientation.y = quaternion.y();
        imu_data.orientation.z = quaternion.z();
        imu_data.orientation.w = quaternion.w();

        // Update linear acceleration in imu_data message
        imu_data.linear_acceleration.x = linear_acc_x;
        imu_data.linear_acceleration.y = linear_acc_y;
        imu_data.linear_acceleration.z = linear_acc_z;

        // Update magnetometer data in imu_data message
        imu_data.orientation_covariance[6] = mag_x;
        imu_data.orientation_covariance[7] = mag_y;
        imu_data.orientation_covariance[8] = mag_z;

        // Reset covariance for linear acceleration
        for (int i = 0; i < 9; ++i) {
            imu_data.linear_acceleration_covariance[i] = 0.0;
        }

        imu_data.header.frame_id = "imu_link";
        current_time = ros::Time::now();
        imu_data.header.stamp = current_time;
        imu_pub.publish(imu_data);
        last_time = current_time;

        loop_rate.sleep();
    }

    return 0;
}
