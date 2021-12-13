#include "sdf_collision_detector.h"


SDF_Collision_Detector::SDF_Collision_Detector()
{

}

SDF_Collision_Detector::~SDF_Collision_Detector()
{

}


void SDF_Collision_Detector::init()
{
    //--------------------------------//
    scan_info_.max_virtual_range_ = 3.0; //Maximum range for defining the laser components in the STATE.
    scan_info_.max_real_range_ = 30.0;
    scan_info_.min_real_range_ = 0.45;
    scan_info_.num_ranges_ = 640; //1079;
    scan_info_.angle_range_ = 115;
//    scan_info_.num_ranges_ = 720; //1079;
//    scan_info_.angle_range_ = 270;
   // scan_info_.sampling_factor_ = int(752/10.0); //72; //Sampling every 27 degrees
    scan_info_.sampling_factor_ = 64; //Sampling every 18 degrees
    scan_info_.angle_sampling_factor_ = scan_info_.angle_range_ / (scan_info_.num_ranges_/scan_info_.sampling_factor_);
    scan_info_.min_range_reset_value_ = 0.3;
    scan_info_.laser_state_normalization_factor_ = scan_info_.max_virtual_range_ - scan_info_.min_range_reset_value_;
    scan_image_info_.angles_ranges_.clear();
    scan_image_info_.laser_scans_image_size_ = cv::Size(300, 300);
    scan_image_info_.angle_ini_ = -57.5;
//    scan_image_info_.angle_ini_ = -135;
    scan_image_info_.angle_ini_rad_ = scan_image_info_.angle_ini_*M_PI/180;
    scan_image_info_.angle_increment_ = scan_info_.angle_range_ / scan_info_.num_ranges_;
    scan_image_info_.p_origin_ = cv::Point(scan_image_info_.laser_scans_image_size_.width/2.0,
                                           scan_image_info_.laser_scans_image_size_.height/2.0);
    for(int i=0;i<scan_info_.num_ranges_;i++)
    {
        float angle_i = (scan_image_info_.angle_ini_ + i*scan_image_info_.angle_increment_) * M_PI/180.0;
        float cos_angle_i = std::cos(angle_i);
        float sin_angle_i = std::sin(angle_i);
        scan_image_info_.angles_ranges_.push_back(angle_i);
        scan_image_info_.cos_angles_ranges_.push_back(cos_angle_i);
        scan_image_info_.sin_angles_ranges_.push_back(sin_angle_i);
    }

    scan_image_info_.laser_scans_image_res_ = scan_info_.max_virtual_range_ / (scan_image_info_.laser_scans_image_size_.width/2.0);
    scan_image_info_.laser_scans_image_ = cv::Mat(scan_image_info_.laser_scans_image_size_.height,
                                                  scan_image_info_.laser_scans_image_size_.width,
                                                  CV_8U, cv::Scalar(0));
    scan_image_info_.obstacles_boundary_image_ = scan_image_info_.laser_scans_image_.clone();
    std::string param_string;
    ros::param::get("~configs_path", param_string);
    if(param_string.length() == 0)
    {
        ROS_ERROR("FATAL: Topic not found");
        return;
    }
    std::string configs_file_name = param_string;
    std::cout<<"++++++++++ CONFIGS PATH ++++++++++"<<std::endl<<configs_file_name<<std::endl;
    bool signal = ReadConfigs(configs_file_name);


}


bool SDF_Collision_Detector::ReadConfigs(std::string &configFile)
{
    pugi::xml_document doc;
    std::ifstream nameFile(configFile.c_str());
    pugi::xml_parse_result result = doc.load(nameFile);

    if(!result)
    {
        std::cout << "ERROR: Could not load the file: " << result.description() << std::endl;
        return 0;
    }

    pugi::xml_node Configuration = doc.child("Laser_Based_Navigation_Config");
    std::string readingValue;

    readingValue = Configuration.child("laser_info").child_value("max_virtual_range");
    scan_info_.max_virtual_range_ = atof(readingValue.c_str());
    std::cout<<"laser_info_.max_virtual_range_: "<<scan_info_.max_virtual_range_<<std::endl;


    readingValue = Configuration.child("laser_info").child_value("max_real_range");
    scan_info_.max_real_range_ = atof(readingValue.c_str());
    std::cout<<"laser_info_.max_real_range_: "<<scan_info_.max_real_range_<<std::endl;


    readingValue = Configuration.child("laser_info").child_value("min_real_range");
    scan_info_.min_real_range_ = atof(readingValue.c_str());
    std::cout<<"laser_info_.min_real_range_: "<<scan_info_.min_real_range_<<std::endl;


    readingValue = Configuration.child("laser_info").child_value("angle_range");
    scan_info_.angle_range_ = atof(readingValue.c_str());
    std::cout<<"laser_info_.angle_range_: "<<scan_info_.angle_range_<<std::endl;


    readingValue = Configuration.child("laser_info").child_value("num_ranges");
    scan_info_.num_ranges_ = atoi(readingValue.c_str());
    std::cout<<"laser_info_.num_ranges_: "<<scan_info_.num_ranges_<<std::endl;


    readingValue = Configuration.child("laser_info").child_value("min_range_reset_value");
    scan_info_.min_range_reset_value_ = atof(readingValue.c_str());
    std::cout<<"laser_info_.min_range_reset_value_: "<<scan_info_.min_range_reset_value_<<std::endl;

    scan_info_.laser_state_normalization_factor_ = scan_info_.max_virtual_range_ - scan_info_.min_range_reset_value_;

    scan_info_.laser_state_normalization_factor_ = scan_info_.max_virtual_range_ - scan_info_.min_range_reset_value_;

    scan_image_info_.angles_ranges_.clear();
    scan_image_info_.laser_scans_image_size_ = cv::Size(300, 300);
    scan_image_info_.angle_ini_ = -57.5;
   // scan_image_info_.angle_ini_ = 45;
    scan_image_info_.angle_ini_rad_ = scan_image_info_.angle_ini_*M_PI/180;
    scan_image_info_.angle_increment_ = scan_info_.angle_range_ / scan_info_.num_ranges_;
    scan_image_info_.p_origin_ = cv::Point(scan_image_info_.laser_scans_image_size_.width/2.0,
                                           scan_image_info_.laser_scans_image_size_.height/2.0);
    for(int i=0;i<scan_info_.num_ranges_;i++)
    {
        float angle_i = (scan_image_info_.angle_ini_ + i*scan_image_info_.angle_increment_) * M_PI/180.0;
        float cos_angle_i = std::cos(angle_i);
        float sin_angle_i = std::sin(angle_i);
        scan_image_info_.angles_ranges_.push_back(angle_i);
        scan_image_info_.cos_angles_ranges_.push_back(cos_angle_i);
        scan_image_info_.sin_angles_ranges_.push_back(sin_angle_i);
    }

    scan_image_info_.laser_scans_image_res_ = scan_info_.max_virtual_range_ / (scan_image_info_.laser_scans_image_size_.width/2.0);
    scan_image_info_.laser_scans_image_ = cv::Mat(scan_image_info_.laser_scans_image_size_.height,
                                                  scan_image_info_.laser_scans_image_size_.width,
                                                  CV_8U, cv::Scalar(255));
    scan_image_info_.obstacles_boundary_image_ = scan_image_info_.laser_scans_image_.clone();

    return 1;

}


//-----------------------------------------------------------------sdf_from_laser--------------------------------------------------------------------------------
void SDF_Collision_Detector::SdffromScan()
{

    if(1)
    {
        std::vector<float> all_saturated_ranges;
        all_saturated_ranges.reserve(scan_info_.laser_ranges_.size());
        for(size_t i=0;i<scan_info_.laser_ranges_.size();i++)
        {
            if(std::isfinite(scan_info_.laser_ranges_[i]))
            {
                if(scan_info_.laser_ranges_[i] > scan_info_.max_virtual_range_)
                    all_saturated_ranges.push_back(scan_info_.max_virtual_range_);
                else
                    all_saturated_ranges.push_back(scan_info_.laser_ranges_[i]);
            }
            else
                all_saturated_ranges.push_back(scan_info_.max_virtual_range_);
        }


        cv::Mat I_all_saturated_ranges = scan_image_info_.laser_scans_image_.clone();
        cv::cvtColor(I_all_saturated_ranges, I_all_saturated_ranges, CV_GRAY2BGR);
        for(size_t i=0;i<all_saturated_ranges.size();i++)
        {
            float delta_pos_proyec_module = all_saturated_ranges[i]/scan_image_info_.laser_scans_image_res_;
            cv::Point2f delta_pos_proyec;
            delta_pos_proyec.x = delta_pos_proyec_module * scan_image_info_.cos_angles_ranges_[i];
            delta_pos_proyec.y = -delta_pos_proyec_module * scan_image_info_.sin_angles_ranges_[i];

            int x_im = scan_image_info_.laser_scans_image_.cols/2 + delta_pos_proyec.x;
            int y_im = scan_image_info_.laser_scans_image_.rows/2 + delta_pos_proyec.y;

            cv::Point p_fin = cv::Point(x_im, y_im);
            cv::line(I_all_saturated_ranges, scan_image_info_.p_origin_, p_fin, cv::Scalar(0), 1);
            if((i%scan_info_.sampling_factor_ == 0) || (i == all_saturated_ranges.size()-1))
                cv::line(I_all_saturated_ranges, scan_image_info_.p_origin_, p_fin, cv::Scalar(255, 0, 0), 2);
        }
       cv::transpose(I_all_saturated_ranges, I_all_saturated_ranges);
       cv::flip(I_all_saturated_ranges, I_all_saturated_ranges, 0);

       cv::imshow("All saturated ranges", I_all_saturated_ranges);

//        cv::imwrite("/home/liang/Downloads/I_all_saturated_ranges.png", I_all_saturated_ranges);
//        cv::waitKey(1);

    }

    cv::Mat I_obstacles_boundary = scan_image_info_.obstacles_boundary_image_.clone();
    int circle_radius = 10;
    for(size_t i=0;i<scan_info_.laser_ranges_.size();i++)
    {
        float delta_pos_proyec_module = scan_info_.laser_ranges_[i]/scan_image_info_.laser_scans_image_res_;
        cv::Point2f delta_pos_proyec;
        delta_pos_proyec.x = delta_pos_proyec_module * scan_image_info_.cos_angles_ranges_[i];
        delta_pos_proyec.y = -delta_pos_proyec_module * scan_image_info_.sin_angles_ranges_[i];

        int x_im = scan_image_info_.laser_scans_image_.cols/2 + delta_pos_proyec.x;
        int y_im = scan_image_info_.laser_scans_image_.rows/2 + delta_pos_proyec.y;

        cv::Point p_range_i = cv::Point(x_im, y_im);
        // cv::Point p_range_i = cv::Point(y_im, x_im);
        cv::circle(I_obstacles_boundary, p_range_i, circle_radius, cv::Scalar(0), -1);
    }

    //Find contours in image in order to compute the number of obstacles
    std::vector<std::vector<cv::Point> > contours;
    cv::Mat I_obstacles_vector_rgb = I_obstacles_boundary.clone();
    cv::bitwise_not(I_obstacles_vector_rgb, I_obstacles_vector_rgb);
    cv::Mat I_for_contours = I_obstacles_vector_rgb.clone();
    cv::findContours(I_for_contours, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

    //Remove those obstacles that are very small (only consider obstacles > than a predefined area)
    std::vector<std::vector<cv::Point> >::iterator itc = contours.begin();
    itc = contours.begin();
    while(itc!=contours.end())
    {
        cv::Rect box = cv::boundingRect(cv::Mat(*itc));
        if(box.area() < 400)
            itc = contours.erase(itc);
        else
            ++itc;
    }

    cv::cvtColor(I_obstacles_vector_rgb, I_obstacles_vector_rgb, CV_GRAY2RGB);

    cv::transpose(I_obstacles_vector_rgb, I_obstacles_vector_rgb);
    cv::flip(I_obstacles_vector_rgb, I_obstacles_vector_rgb, 0);



    cv::Mat dist1, dist2,dview,dist_signed,dis_signed_imshow,dview_uchar;
    cv::Mat temporal_image,dist_signed_pic;

    int maskSize0 = cv::DIST_MASK_PRECISE;
    int distType0 = cv::DIST_L2;
    cv::cvtColor(I_obstacles_vector_rgb, temporal_image, CV_RGB2GRAY);

    //    //distanceTransform
   // cv::distanceTransform( temporal_image, dist1, distType0, maskSize0 );
    cv::distanceTransform( 255-temporal_image, dist2, distType0, maskSize0 );

   // dist_signed_pic=dist2-dist1;
    dist_signed_pic = dist2;
    double minv = 0.0, maxv = 0.0;
    double* minp = &minv;
    double* maxp = &maxv;
    cv::minMaxIdx(dist_signed_pic,minp,maxp);

    if(maxv-minv==0&&maxv<0)
    {
        dist_signed = dist_signed_pic*0;
        dist_signed_real = -FLT_MAX*(dist_signed+1);
    }
    if(maxv-minv==0&&maxv>0)
    {
        dist_signed =dist_signed_pic*0+1;
        dist_signed_real = FLT_MAX*(dist_signed);
    }
    if(maxv-minv>0)
    {
        dist_signed = (dist_signed_pic-minv)/(maxv-minv);
        dist_signed_real = dist_signed_pic*scan_image_info_.laser_scans_image_res_;//0.02m is the resolution of the picture
    }

    //imshow the sdf map
    dis_signed_imshow = dist_signed;
    dis_signed_imshow.convertTo(dview_uchar,CV_8U,255);
    cv::applyColorMap(dview_uchar,dview,cv::COLORMAP_HOT);
    cv::imshow("temporal_image", temporal_image);
    cv::imshow("sdf_map", dview);
    //cv::imwrite("/home/liang/Downloads/temporal_image.png", temporal_image);
    //cv::imwrite("/home/liang/Downloads/dview.png", dview);
    cv::waitKey(1);


}

double SDF_Collision_Detector::Get_Dist_fromSDF(double x_real, double y_real, geometry_msgs::Pose drone_current_pos)
{
    double image_point_y,image_point_x;
    double obs_distance,roll_,pitch_,yaw_;

    tf::Quaternion q(drone_current_pos.orientation.x, drone_current_pos.orientation.y, drone_current_pos.orientation.z, drone_current_pos.orientation.w);
    tf::Matrix3x3 m(q);
    m.getRPY(roll_, pitch_, yaw_);

    Eigen::Vector3d robot_frame,robot_frame_0;
    Eigen::Vector3d global_frame,global_frame_0;

    Eigen::Matrix3d RotationMat;

    global_frame(0) = drone_current_pos.position.x;
    global_frame(1) = drone_current_pos.position.y;
    global_frame(2) = drone_current_pos.position.z;

    global_frame_0(0) = x_real;
    global_frame_0(1) = y_real;
    global_frame_0(2) = drone_current_pos.position.z;

//    RotationMat(0,0) = cos(yaw_);
//    RotationMat(1,0) = -sin(yaw_);
//    RotationMat(2,0) = 0;

//    RotationMat(0,1) = sin(yaw_);
//    RotationMat(1,1) = cos(yaw_);
//    RotationMat(2,1) = 0;

    RotationMat(0,0) = -sin(yaw_);
    RotationMat(1,0) = cos(yaw_);
    RotationMat(2,0) = 0;

    RotationMat(0,1) = cos(yaw_);
    RotationMat(1,1) = sin(yaw_);
    RotationMat(2,1) = 0;

    RotationMat(0,2) = 0;
    RotationMat(1,2) = 0;
    RotationMat(2,2) = 1;

    robot_frame = RotationMat.inverse()*global_frame;
    robot_frame_0 = RotationMat.inverse()*global_frame_0;

    //transfer from real position to image pixel
    image_point_y = 1/scan_image_info_.laser_scans_image_res_*(robot_frame(1) - robot_frame_0(1))
            +scan_image_info_.laser_scans_image_size_.height/2;
    image_point_x = 1/scan_image_info_.laser_scans_image_res_*(robot_frame(0) - robot_frame_0(0))
            +scan_image_info_.laser_scans_image_size_.height/2;
    //obs_distance from signed distance field
    double x_sdf = robot_frame_0(1) - robot_frame(1);
    double y_sdf = robot_frame_0(0) - robot_frame(0);

   // std::cout << "x_real == " << x_sdf << "y_real ==" << y_sdf << "yaw ==" << yaw_<< std::endl;


    if(x_sdf == 0)
    {
      obs_distance = dist_signed_real.at<float>(round(image_point_y),round(image_point_x));
    }
    else
    {
        double sectorangle = 0;
        double angle_ran = scan_info_.angle_range_*(3.14/180);
        Vector2d P1(cos(sectorangle), sin(sectorangle));
        Vector2d P2_0(x_sdf, y_sdf);
        double norm_P2 = sqrt(pow(P2_0.x(), 2) + pow(P2_0.y(), 2));
        Vector2d P2 = P2_0 / norm_P2;
        double theta = atan2(P2.y(),P2.x()) - atan2(P1.y(), P1.x());

        //std::cout << "dist ==" << norm_P2 << "theta ==" << theta << std::endl;

        if(norm_P2 <= scan_info_.max_virtual_range_&& (theta <= angle_ran
                || theta >= -angle_ran)
                )
        {
            obs_distance = dist_signed_real.at<float>(round(image_point_y),round(image_point_x));

            //std::cout << "obs_dist ==" << obs_distance << std::endl;
        }
        else
        {
            obs_distance = 2;
        }
    }

    return obs_distance;
}

bool SDF_Collision_Detector::checkCollision(Eigen::Vector2d &start, Eigen::Vector2d &end, double eta, nav_msgs::Odometry drone_odom_now)
{
    geometry_msgs::Pose drone_current_pose;
    drone_current_pose.position.x    = drone_odom_now.pose.pose.position.x;
    drone_current_pose.position.y    = drone_odom_now.pose.pose.position.y;
    drone_current_pose.position.z    = drone_odom_now.pose.pose.position.z;
    drone_current_pose.orientation.x = drone_odom_now.pose.pose.orientation.x;
    drone_current_pose.orientation.y = drone_odom_now.pose.pose.orientation.y;
    drone_current_pose.orientation.z = drone_odom_now.pose.pose.orientation.z;
    drone_current_pose.orientation.w = drone_odom_now.pose.pose.orientation.w;
    RRTSTAR rrt_star;
    std::vector<double> xnear;
    std::vector<double> xnew;
    double p1_x = start.x();
    double p1_y = start.y();
    double p2_x = end.x();
    double p2_y = end.y();
    xnear.push_back(p1_x);
    xnear.push_back(p1_y);
    xnew.push_back(p2_x);
    xnew.push_back(p2_y);
    double dist = Get_Dist_fromSDF(xnear[0],xnear[1],drone_current_pose);
//    std::cout << "obs_dist ==" << dist << "eta ==" << eta<< std::endl;
    //hu 0.2 firfly 0.3
    if (dist > eta + 0.05)
    {
        return false;
    }
    if(dist < 0.05)
    {
        return true;
    }
    std::vector<double> new_point_;
    new_point_ = xnear;
    char obs = 0;
    while(dist<eta+0.05)
    {

        dist = dist -0.05;
        new_point_ = rrt_star.RRTSteer(new_point_,xnew,dist);
        dist = Get_Dist_fromSDF(new_point_[0],new_point_[1],drone_current_pose);
//        std::cout << "obs_dist ==" << dist << std::endl;
        eta = rrt_star.RRTNorm(new_point_,xnew);
        if(dist < 0.05) {obs=1; break;}
        if(eta == 0 && dist< 0.05)
        {
            obs = 1;
            break;
        }
    }
    if(obs == 1)
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool SDF_Collision_Detector::checkCollision_pt(Vector3d &pt, nav_msgs::Odometry drone_odom_now)
{
    geometry_msgs::Pose drone_current_pose;
    drone_current_pose.position.x    = drone_odom_now.pose.pose.position.x;
    drone_current_pose.position.y    = drone_odom_now.pose.pose.position.y;
    drone_current_pose.position.z    = drone_odom_now.pose.pose.position.z;
    drone_current_pose.orientation.x = drone_odom_now.pose.pose.orientation.x;
    drone_current_pose.orientation.y = drone_odom_now.pose.pose.orientation.y;
    drone_current_pose.orientation.z = drone_odom_now.pose.pose.orientation.z;
    drone_current_pose.orientation.w = drone_odom_now.pose.pose.orientation.w;


    std::vector<double> x;
    double p1_x = pt.x();
    double p1_y = pt.y();
    x.push_back(p1_x);
    x.push_back(p1_y);

    double dist = Get_Dist_fromSDF(x[0],x[1],drone_current_pose);

    //std::cout << "dist ==" << dist << std::endl;

    if(dist > 0.1)
        return false;
    else
        return true;


}
