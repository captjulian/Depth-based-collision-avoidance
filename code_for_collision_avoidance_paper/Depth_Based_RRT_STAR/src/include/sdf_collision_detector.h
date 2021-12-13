#ifndef SDF_COLLISION_DETECTOR_H

#define SDF_COLLISION_DETECTOR_H


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
//ros
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
//opencv
#include <opencv/cv.h>
#include <opencv/cv.hpp>
#include <opencv/highgui.h>
#include "opencv2/core/core.hpp"
//rrt_star
#include "rrtstar.h"
//tf
#include "tf/transform_datatypes.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"


class SDF_Collision_Detector
{

public:
    SDF_Collision_Detector();
    ~SDF_Collision_Detector();

    void init();

    void open(ros::NodeHandle n);

    void run();

    double Get_Dist_fromSDF(double x_real, double y_real, geometry_msgs::Pose drone_current_pos);

    void SdffromScan();

    bool checkCollision(Eigen::Vector2d &start, Eigen::Vector2d &end, double eta, nav_msgs::Odometry drone_odom_now);
    bool checkCollision_pt(Eigen::Vector3d &pt, nav_msgs::Odometry drone_odom_now);

    cv::Mat dist_signed_real;

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

private:


    //--------------read config_file----------------------//
    bool ReadConfigs(std::string &configFile);

    double random_initial_x;
    double random_goal_x;
    double random_up_lim_y;
    double random_low_lim_y;
    double random_up_lim_y_initial;
    double random_low_lim_y_initial;



};

#endif
