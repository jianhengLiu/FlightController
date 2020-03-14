#include "flightController.h"

#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64MultiArray.h>

using namespace std;

PayloadController controller;

void callbackOdometry(const nav_msgs::OdometryConstPtr &odom)
{
    Eigen::Vector3d positionBody(odom->pose.pose.position.x, odom->pose.pose.position.y, odom->pose.pose.position.z);
    Eigen::Vector3d linearVelocity(odom->twist.twist.linear.x, odom->twist.twist.linear.y, odom->twist.twist.linear.z);

    controller.positionBody = positionBody;
    controller.velocityBody = linearVelocity;
}

void callbackImu(const sensor_msgs::ImuConstPtr &imu)
{

    Eigen::Quaterniond quaternion(imu->orientation.w, imu->orientation.x, imu->orientation.y, imu->orientation.z);
    Eigen::Vector3d linearAcceleration(imu->linear_acceleration.x, imu->linear_acceleration.y,
                                       imu->linear_acceleration.z);
    Eigen::Vector3d omega(imu->angular_velocity.x, imu->angular_velocity.y, imu->angular_velocity.z);

    controller.acceleration = linearAcceleration;
    controller.cAngularVelocity_body = omega;
    controller.rotationMatrix_BuW = quaternion.toRotationMatrix();

}

void callbackTarget(const geometry_msgs::PoseConstPtr &pose)
{
    Eigen::Vector3d positionTarget(pose->position.x, pose->position.y, pose->position.z);
    Eigen::Quaterniond quaternionTarget(pose->orientation.w, pose->orientation.x, pose->orientation.y,
                                        pose->orientation.z);

    controller.updateTargetStates(positionTarget, quaternionTarget);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "tracker");
    ros::NodeHandle nh;
    ros::Rate loop_rate(100);

    ros::Subscriber subIMU = nh.subscribe("/imu", 1, callbackImu);
    ros::Subscriber subOdometry = nh.subscribe("/odom", 1, callbackOdometry);
    ros::Subscriber subTarget = nh.subscribe("/target", 1, callbackTarget);

    //多线程订阅
    ros::AsyncSpinner spinner(4); // Use 4 threads

    ros::Publisher pubRotorRevs = nh.advertise<std_msgs::Float64MultiArray>("/rotorRevs", 1);

    double massQuadcopter = 1;
    controller.initializeParameter(massQuadcopter);

    while (ros::ok())
    {
        {

            Eigen::Vector4d revs = controller.getRevs();

            std_msgs::Float64MultiArray revsArray;
            revsArray.data = {revs.x(), revs.y(), revs.z(), revs.w()};

            pubRotorRevs.publish(revsArray);
        }

        controller.updatePastStates();

        spinner.start();
    }

    ros::waitForShutdown();
    return 0;
}


