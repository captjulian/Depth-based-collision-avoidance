//I/O Stream
//std::cout
#include <iostream>

#include <string>

#include "fast_replan_using_sdf_real_flight.h"
#include "ros/ros.h"

int main(int argc, char **argv)
{
    //Init
    ros::init(argc, argv, "fast_planner_using_sdf_rf_node"); //Say to ROS the name of the node and the parameters

    Fast_Replan_Using_SDF_RF fast_replan;

    fast_replan.open(fast_replan.my_node);


     ros::Rate r(30);


    while(ros::ok())
    {
        ros::spinOnce();

        r.sleep();
    }

    return 1;

}
