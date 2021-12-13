#ifndef FAST_REPLAN_USING_SDF_REAL_FLIGHT_H

#define FAST_REPLAN_USING_SDF_REAL_FLIGHT_H


//rrt_star
#include "rrtstar.h"
#include "sdf_collision_detector.h"

#include "pugixml.hpp"
#include "limits.h"
//stl
#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <vector>
#include <iostream>
#include <fstream>
//eigen
#include <Eigen/Eigen>
#include<Eigen/Core>
//opencv
#include <opencv/cv.h>
#include <opencv/cv.hpp>
#include <opencv/highgui.h>
#include "opencv2/core/core.hpp"

//droneMsgROS
#include <droneMsgsROS/dronePose.h>
#include <droneMsgsROS/dronePositionTrajectoryRefCommand.h>
#include <droneMsgsROS/dronePositionRefCommandStamped.h>
//ros
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/Twist.h"
#include "visualization_msgs/MarkerArray.h"
#include "visualization_msgs/Marker.h"

//rrt_star
#include "rrtstar.h"

//tf
#include "tf/transform_datatypes.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"

//Gazebo messages
#include "gazebo_msgs/ModelStates.h"
#include "gazebo_msgs/ModelState.h"
#include "gazebo_msgs/SetModelState.h"



class Fast_Replan_Using_SDF_RF
{
public:
    Fast_Replan_Using_SDF_RF();
    ~Fast_Replan_Using_SDF_RF();


    ros::NodeHandle my_node;
    //timer
    ros::Timer _exec_timer;
    ros::Time time_traj_start;

    double altitude;
    double _Acc;
    double _Vel;
    bool dynamic_env;

    int current_leg_;

    vector<Node *> short_path;

    void execCallback(const ros::TimerEvent &e);

    //RRT and SDF class
    RRTSTAR rrtstar;
    SDF_Collision_Detector sdf_cd;

    double my_obs_dist;

    //Callback
    void Depth_to_ScanCallback(const sensor_msgs::LaserScan &msg);

    void dronePosCallback(const nav_msgs::Odometry &msg);

    void PoseVelocityCallback(const gazebo_msgs::ModelStates::ConstPtr& msg);

    double Xmin;
    double Xmax;
    double Ymin;
    double Ymax;
    double v_max;
    double a_max;
    double END_X;
    double END_Y;
    double END_Z;
    double RRTSTAR_NEIGHBOR_FACTOR;
    double random_initial_x;
    double random_goal_x;
    double random_up_lim_y;
    double random_low_lim_y;
    double random_up_lim_y_initial;
    double random_low_lim_y_initial;

    std::ofstream f_data_recorder;//record data
    std::string file_name;


    void init();
    void run();
    void open(ros::NodeHandle n);

    //drone_state
    enum STATE {
      INIT,
      WAIT_TARGET,
      GEN_NEW_TRAJ,
      EXEC_TRAJ,
      REPLAN_TRAJ
    } exec_state = STATE::INIT;

    void changeState(STATE new_state, string pos_call);

    void printState();


    //rrt_star
    vector<Node *> rrt_star_path_search(nav_msgs::Odometry drone_odom_now, geometry_msgs::Pose end_point);

    double GetRandomReal(int low, int up);

    vector<Node *> path_short(vector<Node *> rrt_path, nav_msgs::Odometry drone_odom_now);

    void drone_position_control(Eigen::Vector3d pos);

    bool pathGeneration();

    //visuial path
    void publish_rrt_path(vector<Node *> path_);


    //params
    nav_msgs::Odometry drone_odom;

    Eigen::Vector3d target_pt,odom_pt,start_pt;

    Eigen::Vector3d odom_vel, start_vel;

    double replan_thresh, no_replan_thresh;

    bool has_odom, has_target, reach_sub_goal;

    double time_duration;


private:

    //sub
    ros::Subscriber scan_sub;
    ros::Subscriber drone_pos_subs_;
    ros::Subscriber uav_pose_velocity_subs_;

    //pub
    ros::Publisher rrt_path_pub;
    ros::Publisher pose_control_pub;

    //service
    ros::ServiceClient gazebo_set_model_state_srv_;

    // for planning

    bool receive_msg;
    bool re_plan_flag;
    float min_range_t;

    geometry_msgs::Pose start_pos;
    geometry_msgs::Pose end_pos;
    geometry_msgs::Pose drone_pos;
    geometry_msgs::Twist drone_vel;


};

#endif
