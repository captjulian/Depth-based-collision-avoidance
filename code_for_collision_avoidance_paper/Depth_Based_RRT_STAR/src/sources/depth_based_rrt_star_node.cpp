//I/O Stream
//std::cout
#include <iostream>

#include <string>

#include "depth_based_rrt_star.h"
#include "ros/ros.h"

int main(int argc, char **argv)
{
    //Init
    ros::init(argc, argv, "depth_based_rrt_star_node"); //Say to ROS the name of the node and the parameters

    Depth_BASED_RRT_STAR depth_rrt_star;

    depth_rrt_star.setUp();

     ros::Rate r(30);


    while(ros::ok())
    {
        depth_rrt_star.run();
        ros::spinOnce();

        r.sleep();
    }

    return 1;

}
