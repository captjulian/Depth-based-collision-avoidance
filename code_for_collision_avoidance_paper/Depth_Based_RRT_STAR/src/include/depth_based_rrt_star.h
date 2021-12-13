#ifndef Depth_BASED_RRT_STAR_H

#define Depth_BASED_RRT_STAR_H

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
//robotprocess
#include "robot_process.h"
//droneMsgROS
#include "trajectory_msgs/MultiDOFJointTrajectory.h"
#include "trajectory_msgs/MultiDOFJointTrajectoryPoint.h"
#include <droneMsgsROS/dronePose.h>
#include <droneMsgsROS/dronePositionTrajectoryRefCommand.h>
//ros
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/Twist.h"
//tra_generate
#include <mav_trajectory_generation/polynomial_optimization_linear.h>
#include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>
#include <mav_trajectory_generation/trajectory.h>
#include <mav_trajectory_generation_ros/trajectory_sampler_node.h>
#include <mav_trajectory_generation_ros/ros_visualization.h>
//rrt_star
#include "rrtstar.h"
//tf
#include "tf/transform_datatypes.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"

class Depth_BASED_RRT_STAR : public RobotProcess
{
public:
    Depth_BASED_RRT_STAR();
    ~Depth_BASED_RRT_STAR();
    //robot_process_function
    void ownSetUp();
    void ownStart();
    void ownStop();
    void ownRun();
    void init();

    void Depth_to_ScanCallback(const sensor_msgs::LaserScan &msg);

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
    //---------------------------------------------------------------------//
    ros::NodeHandle n;
private:
    //sub
    ros::Subscriber scan_sub;
    ros::Subscriber drone_pos_subs_;
    //pub
    ros::Publisher rrt_path_pub;
    ros::Publisher traj_control_pub;
    ros::Publisher tra_visual_;
    //varies
    cv::Mat dist_signed_real;
    bool receive_msg;
    bool re_plan_flag;
    float min_range_t;

    geometry_msgs::Pose start_pos;
    geometry_msgs::Pose drone_pos;
    geometry_msgs::Twist drone_vel;
    trajectory_msgs::MultiDOFJointTrajectory my_traj;

    struct {
        float max_virtual_range_;
        float max_real_range_, min_real_range_;
        float min_range_reset_value_;
        int num_ranges_;
        int sampling_factor_;
        float angle_range_;
        float angle_sampling_factor_;
        float laser_state_normalization_factor_;

        std::vector<float> laser_ranges_;
        std::vector<float> min_laser_ranges_norm_;
    }scan_info_;

    struct {
        cv::Mat laser_scans_image_, obstacles_boundary_image_;
        cv::Size laser_scans_image_size_;
        float laser_scans_image_res_;
        float angle_ini_;
        float angle_ini_rad_;
        float angle_increment_;
        cv::Point p_origin_;
        std::vector<float> angles_ranges_;
        std::vector<float> cos_angles_ranges_;
        std::vector<float> sin_angles_ranges_;
    }scan_image_info_;

    //--------------read config_file----------------------//
    bool ReadConfigs(std::string &configFile);

    double Get_Dist_fromSDF(double x_real, double y_real, geometry_msgs::Pose drone_current_pos);

    void SdffromScan();

    void trajectory_generate(vector<Node *> path_, geometry_msgs::Pose end,
                             trajectory_msgs::MultiDOFJointTrajectory &my_traj, double v_max, double a_max,
                             geometry_msgs::Twist drone_vel);

    //Callback_function
    void dronePosCallback(const nav_msgs::Odometry &msg);
    //collision_check
    bool checkCollision(Vector2d &start, Vector2d &end, double eta, geometry_msgs::Pose drone_current_pose);

    double RRTNorm(std::vector<double> x1, std::vector<double> x2);
    double RRTsign(double n);
    std::vector<double> RRTSteer(std::vector<double> x_nearest, std::vector<double> x_rand, double eta);

    vector<Node *> rrt_star_perform(geometry_msgs::Pose startpoint,geometry_msgs::Pose endpoint,
                                    geometry_msgs::Pose drone_current_pose);

    vector<Node *> path_short(vector<Node *> rrt_path, geometry_msgs::Pose startpoint, geometry_msgs::Pose endpoint,
                              geometry_msgs::Pose drone_current_pose);

    double GetRandomReal(int low, int up);
    //visuial path
    void publish_rrt_path(vector<Node *> path_, geometry_msgs::Pose endpoint);

    void mav_tra_generation_(std::vector<std::vector<double> > pos, double delta_t,
                             trajectory_msgs::MultiDOFJointTrajectory &my_traj,
                             Eigen::Vector4d current_vel, double vcc_max, double ac_max);

    //pub
    inline void publishTra(trajectory_msgs::MultiDOFJointTrajectory traj_);

};

#endif
