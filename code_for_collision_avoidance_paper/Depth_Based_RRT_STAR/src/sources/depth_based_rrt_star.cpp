#include "depth_based_rrt_star.h"

Depth_BASED_RRT_STAR::Depth_BASED_RRT_STAR()
{

}

Depth_BASED_RRT_STAR::~Depth_BASED_RRT_STAR()
{

}

void Depth_BASED_RRT_STAR::init()
{
    //---------ros param-----------//
    ros::param::get("~Xmin", Xmin);
    if(Xmin == 0)
        Xmin = -3.0;
    ros::param::get("~Xmax", Xmax);
    if(Xmax == 0)
        Xmax = 6.0;
    ros::param::get("~Ymin", Ymin);
    if(Ymin == 0)
        Ymin = -3.0;
    ros::param::get("~Ymax", Ymax);
    if(Ymax == 0)
        Ymax = 3.0;
    ros::param::get("~END_X", END_X);
    if(END_X == 0)
        END_X = 0.0;
    ros::param::get("~END_Y", END_Y);
    if(END_Y == 0)
        END_Y = 0.0;
    ros::param::get("~END_Z", END_Z);
    if(END_Z == 0)
        END_Z = 3.0;
    ros::param::get("~vmax", v_max);
    if(v_max == 0)
        v_max = 3.0;
    ros::param::get("~amax", a_max);
    if(a_max == 0)
        a_max = 3.0;
    ros::param::get("~RRTSTAR_NEIGHBOR_FACTOR", RRTSTAR_NEIGHBOR_FACTOR);
    if(RRTSTAR_NEIGHBOR_FACTOR == 0)
        RRTSTAR_NEIGHBOR_FACTOR = 3.0;
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

    receive_msg = false;
    re_plan_flag = true;

}

void Depth_BASED_RRT_STAR::ownSetUp()
{
    this ->init();
}

void Depth_BASED_RRT_STAR::ownStop()
{

}

void Depth_BASED_RRT_STAR::ownStart()
{
    //sub
    scan_sub = n.subscribe("/depth_transfor/scan", 1, &Depth_BASED_RRT_STAR::Depth_to_ScanCallback, this);
   // scan_sub = n.subscribe("/drone7/scan", 1, &Depth_BASED_RRT_STAR::Depth_to_ScanCallback, this);
    drone_pos_subs_ = n.subscribe("/firefly1/odometry_sensor1/odometry",1,
                                  &Depth_BASED_RRT_STAR::dronePosCallback,this);
    //pub
    rrt_path_pub = n.advertise<visualization_msgs::Marker>("rrt_path",1,true);
    traj_control_pub = n.advertise<trajectory_msgs::MultiDOFJointTrajectory>("/firefly1/command/trajectory", 1);
    tra_visual_ = n.advertise<visualization_msgs::MarkerArray>("trajectory",1,true);

}

void Depth_BASED_RRT_STAR::ownRun()
{

    if(this->getState() == aerostack_msgs::ProcessState::NotStarted)
        return;

//    if(receive_msg == true)
//    {


//    }
//    else
//    {
//        std::cout << "no depth message received !!" << std::endl;
//    }

    if(receive_msg==true && re_plan_flag == true)
    {

        my_traj.points.clear();

        vector<Node *> rrt_path;
        vector<Node *> short_path;

        geometry_msgs::Pose end_pos;

        geometry_msgs::Pose drone_pos_now;
        geometry_msgs::Twist drone_vel_now;
        drone_vel_now.linear.x = drone_vel.linear.x;
        drone_vel_now.linear.y = drone_vel.linear.y;
        drone_vel_now.linear.z = drone_vel.linear.z;
        drone_pos_now.position.x = drone_pos.position.x;
        drone_pos_now.position.y = drone_pos.position.y;
        drone_pos_now.position.z = drone_pos.position.z;
        drone_pos_now.orientation.x = drone_pos.orientation.x;
        drone_pos_now.orientation.y = drone_pos.orientation.y;
        drone_pos_now.orientation.z = drone_pos.orientation.z;
        drone_pos_now.orientation.w = drone_pos.orientation.w;
        start_pos.position.x = drone_pos_now.position.x;
        start_pos.position.y = drone_pos_now.position.y;
        start_pos.position.z = drone_pos_now.position.z;
        end_pos.position.x = END_X;
        end_pos.position.y = END_Y;
        end_pos.position.z = END_Z;

        rrt_path = rrt_star_perform(drone_pos_now, end_pos, drone_pos_now);

        if(rrt_path.size() > 3)
        {
            short_path = path_short(rrt_path,drone_pos_now, end_pos, drone_pos_now);
        }
        else
        {
            short_path = rrt_path;
        }
        publish_rrt_path(short_path, end_pos);
        trajectory_generate(short_path,end_pos,my_traj,v_max,a_max,drone_vel_now);
        rrt_path.clear();
        short_path.clear();
        publishTra(my_traj);

        //re_plan_flag = false;
        re_plan_flag = true;
    }

    if(receive_msg == true)
    {

//            clock_t startTime,endTime;
//            startTime = clock();
        std::vector<geometry_msgs::Point> collision_check_point;
         for(int num_in_traj = 0; num_in_traj< my_traj.points.size(); num_in_traj ++)
         {
            geometry_msgs::Point point_check;
            double x_ = drone_pos.position.x - my_traj.points[num_in_traj].transforms[0].translation.x;
            double y_ = drone_pos.position.y - my_traj.points[num_in_traj].transforms[0].translation.y;
            double z_ = drone_pos.position.z - my_traj.points[num_in_traj].transforms[0].translation.z;
            double dist = sqrt(pow(x_, 2) + pow(y_, 2) + pow(z_,2));
            if(dist < 1.5)
            {
                point_check.x = my_traj.points[num_in_traj].transforms[0].translation.x;
                point_check.y = my_traj.points[num_in_traj].transforms[0].translation.y;
                point_check.z = my_traj.points[num_in_traj].transforms[0].translation.z;
                collision_check_point.push_back(point_check);
            }
         }

         for(int point_check_num = 0; point_check_num < collision_check_point.size(); point_check_num++)
         {
             double dist_drone_to_obs = Get_Dist_fromSDF(collision_check_point[point_check_num].x,
                                                         collision_check_point[point_check_num].y, drone_pos);

             //std::cout << "dist_obs ==" << dist_drone_to_obs << std::endl;
             if(dist_drone_to_obs < 0.15)
             {
                 re_plan_flag =true;
                 break;
             }

         }

         collision_check_point.clear();

//         double dist_x = drone_pos.position.x - start_pos.position.x;
//         double dist_y = drone_pos.position.y - start_pos.position.y;

//         double dist_s_now = sqrt(pow(dist_x, 2) + pow(dist_y, 2));

//         if(dist_s_now > scan_info_.max_virtual_range_)
//         {
//             re_plan_flag =true;
//         }
//             endTime = clock();
//             std::cout << "Totle Time : " <<(double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << std::endl;
    }



}

void Depth_BASED_RRT_STAR::dronePosCallback(const nav_msgs::Odometry &msg)
{
    drone_pos.position.x = msg.pose.pose.position.x;
    drone_pos.position.y = msg.pose.pose.position.y;
    drone_pos.position.z = msg.pose.pose.position.z;
    drone_vel.linear.x = msg.twist.twist.linear.x;
    drone_vel.linear.y = msg.twist.twist.linear.y;
    drone_vel.linear.z = msg.twist.twist.linear.z;
    drone_pos.orientation.w = msg.pose.pose.orientation.w;
    drone_pos.orientation.x = msg.pose.pose.orientation.x;
    drone_pos.orientation.y = msg.pose.pose.orientation.y;
    drone_pos.orientation.z = msg.pose.pose.orientation.z;
}

//Norm function
double Depth_BASED_RRT_STAR::RRTNorm(std::vector<double> x1,std::vector<double> x2)
{
    return pow(	(pow(abs((x2[0]-x1[0])),2)+pow(abs((x2[1]-x1[1])),2))	,0.5);
}

double Depth_BASED_RRT_STAR::RRTsign(double n)
{
    if (n<0.0){return -1.0;}
    else{return 1.0;}
}

std::vector<double> Depth_BASED_RRT_STAR::RRTSteer(std::vector<double> x_nearest ,
                                                  std::vector<double> x_rand, double eta)
{
    std::vector<double> x_new;

    if (RRTNorm(x_nearest,x_rand)<=eta){
        x_new=x_rand;
    }
    else{
        Vector2d to, from;
        from.x() = x_nearest[0];
        from.y() = x_nearest[1];
        to.x() = x_rand[0];
        to.y() = x_rand[1];
        Vector2d intermediate = to - from;
        intermediate = intermediate / intermediate.norm();
        Vector2d pos = from + eta * intermediate;
        x_new.push_back(pos.x());
        x_new.push_back(pos.y());
    }
    return x_new;
}

void Depth_BASED_RRT_STAR::Depth_to_ScanCallback(const sensor_msgs::LaserScan &msg)
{

    scan_info_.laser_ranges_.clear();
    scan_info_.laser_ranges_.insert(scan_info_.laser_ranges_.end(), msg.ranges.begin(), msg.ranges.end());

    //std::cout << "num_range="<<scan_info_.laser_ranges_.size()<<std::endl;
    min_range_t = *std::min_element(scan_info_.laser_ranges_.begin(), scan_info_.laser_ranges_.end());
    receive_msg = true;
    SdffromScan();

}

bool Depth_BASED_RRT_STAR::ReadConfigs(std::string &configFile)
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
void Depth_BASED_RRT_STAR::SdffromScan()
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
//       cv::Point2f center;
//       center.x = scan_image_info_.laser_scans_image_size_.width/2.0;
//       center.y = scan_image_info_.laser_scans_image_size_.height/2.0;
//       cv::Size dst_sz(scan_image_info_.laser_scans_image_size_.width, scan_image_info_.laser_scans_image_size_.height);
//       cv::Mat M_1 = cv::getRotationMatrix2D(center, -30, 1);
//       cv::warpAffine(I_all_saturated_ranges, I_all_saturated_ranges, M_1, dst_sz);
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

double Depth_BASED_RRT_STAR::Get_Dist_fromSDF(double x_real, double y_real, geometry_msgs::Pose drone_current_pos)
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

    RotationMat(0,0) = cos(yaw_);
    RotationMat(1,0) = -sin(yaw_);
    RotationMat(2,0) = 0;

    RotationMat(0,1) = sin(yaw_);
    RotationMat(1,1) = cos(yaw_);
    RotationMat(2,1) = 0;

    RotationMat(0,2) = 0;
    RotationMat(1,2) = 0;
    RotationMat(2,2) = 1;

    robot_frame = RotationMat.inverse()*global_frame;
    robot_frame_0 = RotationMat.inverse()*global_frame_0;
    //transfer from real position to image pixel
    image_point_y = 1/scan_image_info_.laser_scans_image_res_*(robot_frame(0) - robot_frame_0(0))
            +scan_image_info_.laser_scans_image_size_.height/2;
    image_point_x = 1/scan_image_info_.laser_scans_image_res_*(robot_frame(1) - robot_frame_0(1))
            +scan_image_info_.laser_scans_image_size_.height/2;
    //obs_distance from signed distance field
    double x_sdf = robot_frame_0(0) - robot_frame(0);
    double y_sdf = robot_frame_0(1) - robot_frame(1);

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

bool Depth_BASED_RRT_STAR::checkCollision(Vector2d &start, Vector2d &end, double eta, geometry_msgs::Pose drone_current_pose)
{
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
    if (dist > eta + 0.15)
    {
        return true;
    }
    if(dist < 0.15)
    {
        return false;
    }
    std::vector<double> new_point_;
    new_point_ = xnear;
    char obs = 0;
    while(dist<eta+0.15)
    {

        dist = dist -0.15;
        new_point_ = RRTSteer(new_point_,xnew,dist);
        dist = Get_Dist_fromSDF(new_point_[0],new_point_[1],drone_current_pose);
        eta = RRTNorm(new_point_,xnew);
        if(dist < 0.15) {obs=1; break;}
        if(eta == 0 && dist< 0.15)
        {
            obs = 1;
            break;
        }
    }
    if(obs == 1)
    {
        return false;
    }
    else
    {
        return true;
    }
}

vector<Node *> Depth_BASED_RRT_STAR::rrt_star_perform(geometry_msgs::Pose startpoint, geometry_msgs::Pose endpoint, geometry_msgs::Pose drone_current_pose)
{
    RRTSTAR rrtstar;
    rrtstar.setMaxIterations(100);
    rrtstar.setStepSize(1.0);
    rrtstar.setSTART(startpoint.position.x,startpoint.position.y);
    rrtstar.setEND(endpoint.position.x,endpoint.position.y);
    srand (time(NULL));

    for(int i = 0; i <  rrtstar.max_iter; i++)
    {
        Node* q;

        float x1 = (float)rand()/(RAND_MAX + 1.0)*(Xmax-Xmin) + Xmin;
        float y1 = (float)rand()/(RAND_MAX + 1.0)*(Ymax-Ymin) + Ymin;

        Vector2d point(x1, y1);
        float orient = rand()%1000 /1000.0 * 2 * 3.142;

        if ( orient < 2*3.142) {
            q = new Node;
            q->position = point;
            q->orientation = orient;
        }
        else
        {
            q=NULL;
        }
        if ( rrtstar.reached() && checkCollision(rrtstar.lastNode->position, rrtstar.endPos,rrtstar.step_size,drone_current_pose)) {
            cout<<"Reached Destination!"<<endl;
            break;
        }

        if (q) {
            Node *qNearest =  rrtstar.nearest(q->position);
            if ( rrtstar.distance(q->position, qNearest->position) >  rrtstar.step_size) {
                Vector3d newConfigPosOrient;

                newConfigPosOrient = rrtstar.newConfig(q, qNearest);
                Vector2d newConfigPos(newConfigPosOrient.x(), newConfigPosOrient.y());
                if (checkCollision(newConfigPos,qNearest->position,rrtstar.step_size, drone_current_pose)) {

                    Node *qNew = new Node;
                    qNew->position = newConfigPos;
                    qNew->orientation = newConfigPosOrient.z();
                    vector<Node *> Qnear;
                    rrtstar.near(qNew->position,  rrtstar.step_size*RRTSTAR_NEIGHBOR_FACTOR, Qnear);
                    Node *qMin = qNearest;
                    double cmin =  rrtstar.Cost(qNearest) +  rrtstar.PathCost(qNearest, qNew);
                    for(int j = 0; j < Qnear.size(); j++){
                        Node *qNear = Qnear[j];
                        if(checkCollision(qNear->position, qNew->position,rrtstar.step_size,drone_current_pose) &&
                                ( rrtstar.Cost(qNear)+rrtstar.PathCost(qNear, qNew)) < cmin ){
                            qMin = qNear; cmin =  rrtstar.Cost(qNear)+ rrtstar.PathCost(qNear, qNew);
                        }
                    }
                    rrtstar.add(qMin, qNew);
                    for(int j = 0; j < Qnear.size(); j++){
                        Node *qNear = Qnear[j];
                        if(checkCollision(qNew->position, qNear->position,rrtstar.step_size,drone_current_pose) &&
                                ( rrtstar.Cost(qNew)+ rrtstar.PathCost(qNew, qNear)) <  rrtstar.Cost(qNear) ){
                            Node *qParent = qNear->parent;
                            // Remove edge between qParent and qNear
                            qParent->children.erase(std::remove(qParent->children.begin(), qParent->children.end(), qNear), qParent->children.end());
                            // Add edge between qNew and qNear
                            qNear->cost =  rrtstar.Cost(qNew) +  rrtstar.PathCost(qNew, qNear);
                            qNear->parent = qNew;
                            qNew->children.push_back(qNear);
                        }
                    }
                }
            }
        }
    }
    Node *q;
    if ( rrtstar.reached()) {
        q =  rrtstar.lastNode;

    }
    else
    {
        // if not reached yet, then shortestPath will start from the closest node to end point.
        q =  rrtstar.nearest(rrtstar.endPos);
        cout<<"Exceeded max iterations!"<<endl;

    }
    // generate shortest path to destination.
    while (q != NULL) {
        rrtstar.path.push_back(q);
        q = q->parent;
    }

   return rrtstar.path;
}

//path_short
vector<Node *> Depth_BASED_RRT_STAR::path_short(vector<Node *> rrt_path,
                                      geometry_msgs::Pose startpoint, geometry_msgs::Pose endpoint, geometry_msgs::Pose drone_current_pose)
{
    int nbFailures = 0;
    int maxNbFailures = 3;
    vector<Node *> short_cut_path;
    srand (time(NULL));
    while(nbFailures < maxNbFailures)
    {
        bool failure = true;
        int RRT_path_size = rrt_path.size();
        short_cut_path.clear();
        if(RRT_path_size ==2)
        {
            break;
        }
        float x_low = GetRandomReal(1,RRT_path_size-2);
        int X1 = round(x_low);
        Vector2d start0(startpoint.position.x,startpoint.position.y);
        Vector2d end2(endpoint.position.x,endpoint.position.y);
        Vector2d end0(rrt_path[RRT_path_size-1-X1]->position.x(),rrt_path[RRT_path_size-1-X1]->position.y());

        double l01 = pow(	(pow(abs((end0.x()-start0.x())),2)+pow(abs((end0.y()-start0.y())),2))	,0.5);
        double l12 = pow(	(pow(abs((end2.x()-end0.x())),2)+pow(abs((end2.y()-end0.y())),2))	,0.5);
        if(checkCollision(end0,end2,l12, drone_current_pose))
        {

            Node* node1;
            node1 = new Node;
            node1->position.x() = end0.x();
            node1->position.y() = end0.y();
            short_cut_path.push_back(node1);
            failure = false;
        }
        else
        {
            for(int i = 0;i< RRT_path_size-X1-1;i++)
            {
                short_cut_path.push_back(rrt_path[i]);

            }
        }
        if(checkCollision(start0,end0,l01,drone_current_pose))
        {


            Node* node1;
            node1 = new Node;
            node1->position.x() = start0.x();
            node1->position.y() = start0.y();
            short_cut_path.push_back(node1);
            failure = false;

        }
        else
        {
            for(int i = RRT_path_size-X1;i< RRT_path_size;i++)
            {
                short_cut_path.push_back(rrt_path[i]);

            }

        }
        rrt_path.clear();
        for(int j = 0; j<short_cut_path.size(); j++)
        {

            rrt_path.push_back(short_cut_path[j]);

        }

        if(RRT_path_size < 4)
        {

            nbFailures = maxNbFailures;

        }
        else
        {
            if(failure)
            {
                nbFailures = nbFailures + 1;

            }

        }

    }
    return rrt_path;
}

double Depth_BASED_RRT_STAR::GetRandomReal(int low, int up)
{

    double fResult;
    if (low > up)
    {
      int temp = low;
      low = up;
      up = temp;
    }
    fResult = low + rand()%1000 /1000.0*(up - low);
    return fResult;

}

//trajectory_controller
void Depth_BASED_RRT_STAR::trajectory_generate(vector<Node *> path_, geometry_msgs::Pose end,
                                               trajectory_msgs::MultiDOFJointTrajectory &my_traj,
                                               double v_max, double a_max, geometry_msgs::Twist drone_vel)
{
    std::vector<std::vector<double> > traj_;
    std::vector<std::vector<double> > my_traj_;


    for(int i=0; i<path_.size();i++)
    {
        {
            std::vector<double> temp_pos;
            temp_pos.push_back(path_[path_.size()-i-1]->position.x());
            temp_pos.push_back(path_[path_.size()-i-1]->position.y());
            temp_pos.push_back(1.1);

            traj_.push_back(temp_pos);
            temp_pos.clear();
        }
    }

    std::vector<double> temp_pos;
    temp_pos.push_back(end.position.x);
    temp_pos.push_back(end.position.y);
    temp_pos.push_back(1.1);
    temp_pos.push_back(0);
    traj_.push_back(temp_pos);
    temp_pos.clear();

    Eigen::Vector4d current_vel;

    current_vel[0] = drone_vel.linear.x;
    current_vel[1] = drone_vel.linear.y;
    current_vel[2] = drone_vel.linear.z;
    current_vel[3] = 0;

    mav_tra_generation_(traj_,0.2,my_traj,current_vel,v_max,a_max);

    traj_.clear();


}


void Depth_BASED_RRT_STAR::mav_tra_generation_(std::vector<std::vector<double> > pos, double delta_t,
                                                      trajectory_msgs::MultiDOFJointTrajectory &my_traj,
                                                      Eigen::Vector4d current_vel, double vcc_max, double ac_max)
{

    mav_trajectory_generation::Vertex::Vector vertices;
    const int dimension = 4;
    const int derivative_to_optimize = mav_trajectory_generation::derivative_order::SNAP;
    mav_trajectory_generation::Vertex start(dimension), middle(dimension), end(dimension);
    start.makeStartOrEnd(Eigen::Vector4d(pos[0][0], pos[0][1], pos[0][2], 0), derivative_to_optimize);
    //start.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, current_vel);
    //start.addConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, current_acc);
    vertices.push_back(start);
    int pos_size = pos.size()-1;
    //int goal_size = pos.size()-1;
    if(pos_size == 1)
    {
        end.makeStartOrEnd(Eigen::Vector4d(pos[pos_size][0], pos[pos_size][1],
                pos[pos_size][2], 0), derivative_to_optimize);
        vertices.push_back(end);
    }
    else
    {
        for(int i = 1; i < pos_size; i++)
        {
            middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION,
                                 Eigen::Vector4d(pos[i][0], pos[i][1], pos[i][2], 0));

            vertices.push_back(middle);
        }

        end.makeStartOrEnd(Eigen::Vector4d(pos[pos_size][0], pos[pos_size][1],
                pos[pos_size][2], 0), derivative_to_optimize);
        vertices.push_back(end);
    }

    std::vector<double> segment_times;
    const double v_max = vcc_max;
    const double a_max = ac_max;

    const double magic_fabian_constant = 10; // A tuning parameter.
    segment_times = mav_trajectory_generation::estimateSegmentTimesNfabian(vertices, v_max, a_max, magic_fabian_constant);

    mav_trajectory_generation::NonlinearOptimizationParameters parameters;
    parameters.max_iterations = 1000;
    parameters.f_rel = 0.05;
    parameters.x_rel = 0.1;
    parameters.time_penalty = 500.0;
    parameters.initial_stepsize_rel = 0.1;
    parameters.inequality_constraint_tolerance = 0.1;
    mav_trajectory_generation::NonlinearOptimizationParameters::kRichterTimeAndConstraints;

    const int N = 10;
    mav_trajectory_generation::PolynomialOptimizationNonLinear<N> opt(dimension, parameters);
    opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);
    opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::VELOCITY, v_max);
    opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, a_max);

    ROS_INFO("Performing optimization...");
    opt.optimize();

    mav_trajectory_generation::Segment::Vector segments;
    opt.getPolynomialOptimizationRef().getSegments(&segments);
    ROS_INFO("Done.");
    mav_trajectory_generation::Trajectory trajectory;
    opt.getTrajectory(&trajectory);

    mav_msgs::EigenTrajectoryPoint::Vector states;

    double sampling_interval = delta_t;
    bool success = mav_trajectory_generation::sampleWholeTrajectory(trajectory, sampling_interval, &states);
    if(success)
    {
        for(int j = 0; j< states.size() ; j++)
        {
            trajectory_msgs::MultiDOFJointTrajectoryPoint my_point;
            geometry_msgs::Transform position_vector;

            position_vector.translation.x = states[j].position_W[0];
            position_vector.translation.y = states[j].position_W[1];
            position_vector.translation.z = states[j].position_W[2];


//            position_vector.rotation.x = states[j].orientation_W_B.x();
//            position_vector.rotation.y = states[j].orientation_W_B.y();
//            position_vector.rotation.z = states[j].orientation_W_B.z();
//            position_vector.rotation.w = states[j].orientation_W_B.w();

            my_point.transforms.push_back(position_vector);

            geometry_msgs::Twist vel_vector;

            vel_vector.linear.x = states[j].velocity_W[0];
            vel_vector.linear.y = states[j].velocity_W[1];
            vel_vector.linear.z = states[j].velocity_W[2];

//            vel_vector.angular.x = states[j].angular_velocity_W[0];
//            vel_vector.angular.y = states[j].angular_velocity_W[1];
//            vel_vector.angular.z = states[j].angular_velocity_W[2];

            my_point.velocities.push_back(vel_vector);

            geometry_msgs::Twist acc_vector;

            acc_vector.linear.x = states[j].acceleration_W[0];
            acc_vector.linear.y = states[j].acceleration_W[1];
            acc_vector.linear.z = states[j].acceleration_W[2];

//            acc_vector.angular.x = states[j].angular_acceleration_W[0];
//            acc_vector.angular.y = states[j].angular_acceleration_W[1];
//            acc_vector.angular.z = states[j].angular_acceleration_W[2];

            my_point.accelerations.push_back(acc_vector);

            my_traj.points.push_back(my_point);

            my_traj.header.stamp = ros::Time::now();;
            my_traj.points[j].time_from_start = ros::Duration(j*delta_t);

        }

    }
    else
    {
        std::cout << "trajectory generate unsuccessfully !!"<< std::endl;
    }

    visualization_msgs::MarkerArray markers;
    double distance = 0.0; // Distance by which to seperate additional markers. Set 0.0 to disable.
    std::string frame_id = "world";
    // From Trajectory class:

    mav_trajectory_generation::drawMavTrajectory(trajectory, distance, frame_id, &markers);
    tra_visual_.publish(markers);
    markers.markers.clear();

}

inline void Depth_BASED_RRT_STAR::publishTra(trajectory_msgs::MultiDOFJointTrajectory traj_)
{

    //traj_control_pub.publish(traj_);
    //std::cout << "in !!"<<std::endl;
}
//visuial path
void Depth_BASED_RRT_STAR::publish_rrt_path(vector<Node *> path_, geometry_msgs::Pose endpoint)
{
    visualization_msgs::Marker finalpath;

    geometry_msgs::Point point;

    finalpath.header.stamp = ros::Time();
    finalpath.header.frame_id = "world";
    finalpath.ns = "RRT_PATH";

    finalpath.id = 1;
    finalpath.action = visualization_msgs::Marker::ADD;
    finalpath.type = visualization_msgs::Marker::LINE_STRIP;
    finalpath.pose.orientation.w = 1;
    finalpath.scale.x = 0.05;
    finalpath.color.a = 1.0; // Don't forget to set the alpha!
    finalpath.color.r = 1;

    finalpath.color.g = 1;
    finalpath.color.b = 0;

    point.x = endpoint.position.x;
    point.y = endpoint.position.y;
    point.z = 1.25;
    finalpath.points.push_back(point);
    for(int i=0; i<path_.size();i++)
    {

        {
            point.x = path_[i]->position.x();
            point.y = path_[i]->position.y();
            point.z = 1.25;
            finalpath.points.push_back(point);
        }

    }

    rrt_path_pub.publish(finalpath);
}

