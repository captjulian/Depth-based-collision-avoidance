//I/O Stream
//std::cout
#include <iostream>
#include <string>
#include "ros/ros.h"

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>

ros::Publisher gazebo_pos_pub;


void gazebo_pos_callback(const geometry_msgs::Pose &msg)

{
    geometry_msgs::PoseStamped drone_pose;

    drone_pose.header.stamp = ros::Time::now();
    drone_pose.header.frame_id = "map";
    drone_pose.pose = msg;

    gazebo_pos_pub.publish(drone_pose);

}

int main(int argc, char **argv)
{
    //Init
    ros::init(argc, argv, "gazebo_pos_node"); //Say to ROS the name of the node and the parameters
    ros::NodeHandle n; //Este nodo admite argumentos!!

    ros::Subscriber uav_estimate_position = n.subscribe("/firefly1/odometry_sensor1/pose", 1,
                                                        gazebo_pos_callback);

    gazebo_pos_pub = n.advertise<geometry_msgs::PoseStamped>("/drone_pose",1,true);


    ros::Rate r(30);
    //Loop -> Ashyncronous Module
    while(ros::ok())
    {
        ros::spin();
        r.sleep();
    }

    return 1;
}
