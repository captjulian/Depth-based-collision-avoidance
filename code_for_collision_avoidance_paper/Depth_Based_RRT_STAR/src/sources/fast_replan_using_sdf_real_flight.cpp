#include "fast_replan_using_sdf_real_flight.h"

Fast_Replan_Using_SDF_RF::Fast_Replan_Using_SDF_RF()
{


}


Fast_Replan_Using_SDF_RF::~Fast_Replan_Using_SDF_RF()
{

}

void Fast_Replan_Using_SDF_RF::init()
{
    //---------ros param-----------//
    ros::param::get("~Xmin", Xmin);
    if(Xmin == 0)
        Xmin = -3.0;
    ros::param::get("~Xmax", Xmax);
    if(Xmax == 0)
        Xmax = 4.0;
    ros::param::get("~Ymin", Ymin);
    if(Ymin == 0)
        Ymin = -2.2;
    ros::param::get("~Ymax", Ymax);
    if(Ymax == 0)
        Ymax = 2.2;
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
    ros::param::get("thresh_replan", replan_thresh);//1.5
    if(replan_thresh == 0)
        replan_thresh = 1.5;
    ros::param::get("thresh_no_replan", no_replan_thresh);//2.0
    if(no_replan_thresh == 0)
        no_replan_thresh = 2.0;
    ros::param::get("~random_initial_x", random_initial_x);
    if(random_initial_x == 0)
        random_initial_x = 0.0;
    ros::param::get("~random_goal_x", random_goal_x);
    if(random_goal_x == 0)
        random_goal_x = 50;
    ros::param::get("~random_up_lim_y", random_up_lim_y);
    if(random_up_lim_y == 0)
        random_up_lim_y = 35;
    ros::param::get("~random_low_lim_y", random_low_lim_y);
    if(random_low_lim_y == 0)
        random_low_lim_y = 15;
    ros::param::get("~random_up_lim_y_initial", random_up_lim_y_initial);
    if(random_up_lim_y_initial == 0)
        random_up_lim_y_initial = 35;
    ros::param::get("~random_low_lim_y_initial", random_low_lim_y_initial);
    if(random_low_lim_y_initial == 0)
        random_low_lim_y_initial = 15;
    ros::param::get("~file_name", file_name);
    if(file_name == "")
        file_name = "/home/liang/planner_data_record/document.txt";
    ros::param::get("~alt", altitude);
    if(altitude == 0)
        altitude = 1.0;
    ros::param::get("~vel", _Vel);
    if(_Vel == 0)
        _Vel = 1.0;
    ros::param::get("~acc", _Acc);
    if(_Acc == 0)
        _Acc = 1.0;
    ros::param::get("~dynamic_env", dynamic_env);


    target_pt.x() = END_X;
    target_pt.y() = END_Y;
    target_pt.z() = END_Z;


   // has_odom = false;
    has_odom = true;
    has_target = true;
    reach_sub_goal = false;
    receive_msg = false;


}

void Fast_Replan_Using_SDF_RF::open(ros::NodeHandle n)
{
    init();
    sdf_cd.init();
    //sub
    scan_sub = n.subscribe("/depth_transfor/scan", 1, &Fast_Replan_Using_SDF_RF::Depth_to_ScanCallback, this);
   // scan_sub = n.subscribe("/drone7/scan", 1, &Depth_BASED_RRT_STAR::Depth_to_ScanCallback, this);
    drone_pos_subs_ = n.subscribe("/hummingbird1/odometry_sensor1/odometry",1,
                                  &Fast_Replan_Using_SDF_RF::dronePosCallback,this);
    uav_pose_velocity_subs_ = n.subscribe("/gazebo/model_states", 10, &Fast_Replan_Using_SDF_RF::PoseVelocityCallback, this);

    //pub
    rrt_path_pub = n.advertise<visualization_msgs::Marker>("rrt_path",1,true);

    pose_control_pub = n.advertise<droneMsgsROS::dronePositionRefCommandStamped>("/drone7/dronePositionRefs", 1);


    //timer
     _exec_timer = n.createTimer(ros::Duration(0.01), &Fast_Replan_Using_SDF_RF::execCallback, this);

     //service
      gazebo_set_model_state_srv_ = n.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");

}

void Fast_Replan_Using_SDF_RF::run()
{

}

// change the state to the new state
void Fast_Replan_Using_SDF_RF::changeState(STATE new_state, string pos_call) {
  string state_str[5] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "EXEC_TRAJ",
                         "REPLAN_TRAJ"};
  int pre_s = int(exec_state);
  exec_state = new_state;
  cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " +
              state_str[int(new_state)]
       << endl;
}

void Fast_Replan_Using_SDF_RF::printState() {
  string state_str[5] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "EXEC_TRAJ",
                         "REPLAN_TRAJ"};
  cout << "[Clock]: state: " + state_str[int(exec_state)] << endl;
}


void Fast_Replan_Using_SDF_RF::execCallback(const ros::TimerEvent &e)
{
    static int num = 0;
    num++;
    if (num == 100) {
      printState();
      if (!has_odom)
        cout << "no odom." << endl;
      if (!has_target)
        cout << "wait for goal." << endl;
      if(!receive_msg)
        cout << "wait for scan." << endl;
      num = 0;
    }

    switch (exec_state) {
    case INIT: {
      if (!has_odom)
        return;
      if (!has_target)
        return;
      if (!receive_msg)
        return;
      changeState(GEN_NEW_TRAJ, "STATE");
      has_odom = false;
      receive_msg = false;
      break;
    }

    case WAIT_TARGET: {
      if (!has_target)
        return;
      else
        changeState(GEN_NEW_TRAJ, "STATE");
      break;
    }

    case GEN_NEW_TRAJ: {

      bool success = pathGeneration();

      if (success)
        changeState(EXEC_TRAJ, "STATE");
      else
        changeState(GEN_NEW_TRAJ, "STATE");
      break;
    }

    case EXEC_TRAJ: {

        if(short_path.size() > 0)
        {
            odom_pt.x() = drone_odom.pose.pose.position.x;
            odom_pt.y() = drone_odom.pose.pose.position.y;
            odom_pt.z() = drone_odom.pose.pose.position.z;

            double dist_x = short_path[current_leg_]->position.x() - odom_pt.x();
            double dist_y = short_path[current_leg_]->position.y() - odom_pt.y();

            const double dist_to_end = sqrt(pow(dist_x,2)+ pow(dist_y,2));

            if (current_leg_ != short_path.size() - 1 &&
                    dist_to_end < 0.1) {
                if (current_leg_ == 0) {
                    std::cout << "Going to first waypoint... " << std::endl;
                } else {
                    std::cout << "Leg " << current_leg_ << " of "
                              << short_path.size() - 1 << " completed!" << std::endl;

                }
                current_leg_++;
            }
            Eigen::Vector3d pos;

            pos.x() = short_path[current_leg_]->position.x();
            pos.y() = short_path[current_leg_]->position.y();
            pos.z() = altitude;

            drone_position_control(pos);

            Eigen::Vector2d start,end;

            start.x() = odom_pt.x();
            start.y() = odom_pt.y();

            end.x()   = pos.x();
            end.y()   = pos.y();

            bool collision = sdf_cd.checkCollision(start,end, dist_to_end,drone_odom);

            if(collision == true || current_leg_ == short_path.size() -1)
            {
               changeState(REPLAN_TRAJ, "STATE");
            }
        }

      break;
    }
    case REPLAN_TRAJ: {

      bool success = pathGeneration();
      if (success)
        changeState(EXEC_TRAJ, "STATE");
      else
        changeState(GEN_NEW_TRAJ, "STATE");
      break;
    }
    }
}

void Fast_Replan_Using_SDF_RF::dronePosCallback(const nav_msgs::Odometry &msg)
{
    drone_odom.header.frame_id         = msg.header.frame_id;
    drone_odom.header.stamp            = msg.header.stamp;
    drone_odom.header.seq              = msg.header.seq;
    drone_odom.pose.pose.position.x    = msg.pose.pose.position.x;
    drone_odom.pose.pose.position.y    = msg.pose.pose.position.y;
    drone_odom.pose.pose.position.z    = msg.pose.pose.position.z;
    drone_odom.twist.twist.linear.x    = msg.twist.twist.linear.x;
    drone_odom.twist.twist.linear.y    = msg.twist.twist.linear.y;
    drone_odom.twist.twist.linear.z    = msg.twist.twist.linear.z;
    drone_odom.twist.twist.angular.x   = msg.twist.twist.angular.x;
    drone_odom.twist.twist.angular.y   = msg.twist.twist.angular.y;
    drone_odom.twist.twist.angular.z   = msg.twist.twist.angular.z;
    drone_odom.pose.pose.orientation.w = msg.pose.pose.orientation.w;
    drone_odom.pose.pose.orientation.w = msg.pose.pose.orientation.x;
    drone_odom.pose.pose.orientation.w = msg.pose.pose.orientation.y;
    drone_odom.pose.pose.orientation.w = msg.pose.pose.orientation.z;

    has_odom                           = true;
}

void Fast_Replan_Using_SDF_RF::Depth_to_ScanCallback(const sensor_msgs::LaserScan &msg)
{

    sdf_cd.scan_info_.laser_ranges_.clear();
    sdf_cd.scan_info_.laser_ranges_.insert(sdf_cd.scan_info_.laser_ranges_.end(), msg.ranges.begin(), msg.ranges.end());

    //std::cout << "num_range="<<scan_info_.laser_ranges_.size()<<std::endl;
    min_range_t = *std::min_element(sdf_cd.scan_info_.laser_ranges_.begin(), sdf_cd.scan_info_.laser_ranges_.end());

    sdf_cd.SdffromScan();

    geometry_msgs::Pose drone_pose;
//    drone_pose.position.x = drone_odom.pose.pose.position.x;
//    drone_pose.position.y = drone_odom.pose.pose.position.y;
//    drone_pose.position.z = drone_odom.pose.pose.position.z;
//    drone_pose.orientation.x = drone_odom.pose.pose.orientation.x;
//    drone_pose.orientation.y = drone_odom.pose.pose.orientation.y;
//    drone_pose.orientation.z = drone_odom.pose.pose.orientation.z;
//    drone_pose.orientation.w = drone_odom.pose.pose.orientation.w;
    drone_pose.position.x = 0;
    drone_pose.position.y = 0;
    drone_pose.position.z = 1;
    drone_pose.orientation.x = 0;
    drone_pose.orientation.y = 0;
    drone_pose.orientation.z = 0;
    drone_pose.orientation.w = 1;
    double dist_obs = sdf_cd.Get_Dist_fromSDF(drone_pose.position.x, drone_pose.position.y, drone_pose);

    //std::cout << "dist_to_obs ==" << dist_obs +0.2 <<"ground truth ==" << my_obs_dist <<" compare with ground truth =="<<dist_obs +0.2 - my_obs_dist<< std::endl;
    std::cout << "dist_to_obs ==" << dist_obs +0.2 << std::endl;
    receive_msg = true;
}

vector<Node *> Fast_Replan_Using_SDF_RF::path_short(vector<Node *> rrt_path, nav_msgs::Odometry drone_odom_now)
{
    vector<Node *> rrt_path_rebuild;

    for(int rrt_num = 0; rrt_num < rrt_path.size(); rrt_num++)
    {
        rrt_path_rebuild.push_back(rrt_path[rrt_path.size() -1 -rrt_num]);

    }

    int nbFailures = 0;
    int maxNbFailures = 8;
    vector<Node *> short_cut_path;
    srand (time(0));
    while(nbFailures < maxNbFailures)
    {
        bool failure = true;
        int RRT_path_size = rrt_path_rebuild.size();
        short_cut_path.clear();
        if(RRT_path_size ==2)
        {
            break;
        }
        float x_low = GetRandomReal(1,RRT_path_size-2);
        int X1 = round(x_low);

        Vector2d start0(rrt_path_rebuild[0]->position.x(),rrt_path_rebuild[0]->position.y());
        Vector2d end2(rrt_path_rebuild[rrt_path_rebuild.size()-1]->position.x(),rrt_path_rebuild[rrt_path_rebuild.size()-1]->position.y());
        Vector2d end0(rrt_path_rebuild[X1]->position.x(),rrt_path_rebuild[X1]->position.y());

        double l01 = pow(	(pow(abs((end0.x()-start0.x())),2)+pow(abs((end0.y()-start0.y())),2))	,0.5);
        double l12 = pow(	(pow(abs((end2.x()-end0.x())),2)+pow(abs((end2.y()-end0.y())),2))	,0.5);

        short_cut_path.push_back(rrt_path_rebuild[0]);
        if(!sdf_cd.checkCollision(start0,end0,l01,drone_odom_now))
        {

            Node* node1;
            node1 = new Node;
            node1->position.x() = end0.x();
            node1->position.y() = end0.y();
            short_cut_path.push_back(node1);

            X1 = X1+1;

            failure = false;

        }
        else
        {

            for(int i = 1;i< X1;i++)
            {
                short_cut_path.push_back(rrt_path_rebuild[i]);

            }

        }

        if(!sdf_cd.checkCollision(end0,end2,l12, drone_odom_now))
        {

            failure = false;
        }
        else
        {
            for(int i = X1;i< RRT_path_size-1;i++)
            {
                short_cut_path.push_back(rrt_path_rebuild[i]);

            }
        }

        short_cut_path.push_back(rrt_path_rebuild[rrt_path_rebuild.size()-1]);
        rrt_path_rebuild.clear();
        for(int j = 0; j<short_cut_path.size(); j++)
        {
            rrt_path_rebuild.push_back(short_cut_path[j]);

        }

        if(short_cut_path.size() < 4)
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
    return rrt_path_rebuild;
}

vector<Node *> Fast_Replan_Using_SDF_RF::rrt_star_path_search(nav_msgs::Odometry drone_odom_now, geometry_msgs::Pose end_point)
{

    RRTSTAR rrtstar_now;
    rrtstar_now.setMaxIterations(100);
    rrtstar_now.setStepSize(1.0);
    rrtstar_now.setSTART(drone_odom_now.pose.pose.position.x,drone_odom_now.pose.pose.position.y);
    rrtstar_now.setEND(end_point.position.x,end_point.position.y);
    srand (time(NULL));
    int iter = 0;

    rrtstar_now.path.clear();

    for(int i = 0; i <  rrtstar_now.max_iter; i++)
    {
        Node* q;

        float x1 = (float)rand()/(RAND_MAX + 1.0)*(Xmax-Xmin) + Xmin;
        float y1 = (float)rand()/(RAND_MAX + 1.0)*(Ymax-Ymin) + Ymin;

        Eigen::Vector2d point(x1, y1);
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
        if ( rrtstar_now.reached() && !sdf_cd.checkCollision(rrtstar_now.lastNode->position, rrtstar_now.endPos,rrtstar_now.step_size,drone_odom_now)) {

            cout<<"Reached Destination!"<<endl;
            break;
        }

        iter++;

        if (q) {
            Node *qNearest =  rrtstar_now.nearest(q->position);
            if ( rrtstar_now.distance(q->position, qNearest->position) >  rrtstar_now.step_size) {
                Vector3d newConfigPosOrient;

                newConfigPosOrient = rrtstar_now.newConfig(q, qNearest);
                Vector2d newConfigPos(newConfigPosOrient.x(), newConfigPosOrient.y());
                if (!sdf_cd.checkCollision(newConfigPos,qNearest->position,rrtstar_now.step_size, drone_odom_now)) {

                    Node *qNew = new Node;
                    qNew->position = newConfigPos;
                    qNew->orientation = newConfigPosOrient.z();
                    vector<Node *> Qnear;
                    rrtstar_now.near(qNew->position,  rrtstar_now.step_size*RRTSTAR_NEIGHBOR_FACTOR, Qnear);
                    Node *qMin = qNearest;
                    double cmin =  rrtstar_now.Cost(qNearest) +  rrtstar_now.PathCost(qNearest, qNew);
                    for(int j = 0; j < Qnear.size(); j++){
                        Node *qNear = Qnear[j];
                        if(!sdf_cd.checkCollision(qNear->position, qNew->position,rrtstar_now.step_size,drone_odom_now) &&
                                ( rrtstar_now.Cost(qNear)+rrtstar_now.PathCost(qNear, qNew)) < cmin ){
                            qMin = qNear; cmin =  rrtstar_now.Cost(qNear)+ rrtstar_now.PathCost(qNear, qNew);
                        }
                    }
                    rrtstar_now.add(qMin, qNew);
                    for(int j = 0; j < Qnear.size(); j++){
                        Node *qNear = Qnear[j];
                        if(!sdf_cd.checkCollision(qNew->position, qNear->position,rrtstar_now.step_size,drone_odom_now) &&
                                ( rrtstar_now.Cost(qNew)+ rrtstar_now.PathCost(qNew, qNear)) <  rrtstar_now.Cost(qNear) ){
                            Node *qParent = qNear->parent;
                            // Remove edge between qParent and qNear
                            qParent->children.erase(std::remove(qParent->children.begin(), qParent->children.end(), qNear), qParent->children.end());
                            // Add edge between qNew and qNear
                            qNear->cost =  rrtstar_now.Cost(qNew) +  rrtstar_now.PathCost(qNew, qNear);
                            qNear->parent = qNew;
                            qNew->children.push_back(qNear);
                        }
                    }
                }
            }
        }
    }

    Node *q;
    Node *end_p;
    if ( rrtstar_now.reached()) {

        q =  rrtstar_now.lastNode;

        end_p = new Node;

        end_p->parent = q;
        end_p->orientation = 0.0;
        end_p->cost = q->cost;

        end_p->position = rrtstar_now.endPos;

        rrtstar_now.path.push_back(end_p);


    }
    else
    {
        // if not reached yet, then shortestPath will start from the closest node to end point.
        q =  rrtstar_now.nearest(rrtstar_now.endPos);

        end_p = new Node;

        end_p->parent = q;
        end_p->orientation = 0.0;
        end_p->cost = q->cost;
        end_p->position = rrtstar_now.endPos;

        rrtstar_now.path.push_back(end_p);
        cout<<"Exceeded max iterations!"<<endl;

    }

    // generate shortest path to destination.
    while (q != NULL) {

        rrtstar_now.path.push_back(q);
        q = q->parent;
    }

   return rrtstar_now.path;
}

double Fast_Replan_Using_SDF_RF::GetRandomReal(int low, int up)
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

bool Fast_Replan_Using_SDF_RF::pathGeneration()
{

    current_leg_ = 0;
    short_path.clear();

    end_pos.position.x = target_pt.x();
    end_pos.position.y = target_pt.y();
    end_pos.position.z = target_pt.z();


    vector<Node *> path = rrt_star_path_search(drone_odom,end_pos);
    short_path = path_short(path, drone_odom);

//    for(int rrt_num = 0; rrt_num < path.size(); rrt_num++)
//    {
//        short_path.push_back(path[path.size() -1 -rrt_num]);

//        //                std::cout << "path_point ==" << path[path.size() -1 -rrt_num]->position.x() << " "
//        //                          << path[path.size() -1 -rrt_num]->position.y() << std::endl;
//    }

    publish_rrt_path(short_path);

    time_traj_start = ros::Time::now();
    // return if the trajectory generation successes
    if (short_path.size() > 0)
        return true;
    else
        return false;


}

void Fast_Replan_Using_SDF_RF::drone_position_control(Vector3d pos)
{
   droneMsgsROS::dronePositionRefCommandStamped position_cmd;

   position_cmd.position_command.x = pos.x() + 5;
   position_cmd.position_command.y = pos.y();
   position_cmd.position_command.z = pos.z();

  // pose_control_pub.publish(position_cmd);

}

void Fast_Replan_Using_SDF_RF::publish_rrt_path(vector<Node *> path_)
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

void Fast_Replan_Using_SDF_RF::PoseVelocityCallback(const gazebo_msgs::ModelStates::ConstPtr &msg)
{

    cv::Point3f uav_position, uav_linear_vel;
    cv::Point3f obs_position;
    for (int i=0; i<msg->name.size(); i++)
    {
        if (msg->name[i].compare("hummingbird1") == 0)
        {

            uav_position.x = msg->pose[i].position.x;
            uav_position.y = msg->pose[i].position.y;
            uav_position.z = msg->pose[i].position.z;

            uav_linear_vel.x = msg->twist[i].linear.x;
            uav_linear_vel.y = msg->twist[i].linear.y;
            uav_linear_vel.z = msg->twist[i].linear.z;
            break;

        }
        if (msg->name[i].compare("obstacle_cylinder") == 0)
        {
            obs_position.x = msg->pose[i].position.x;
            obs_position.y = msg->pose[i].position.y;
            obs_position.z = msg->pose[i].position.z;
        }

    }

    double delta_dist_ob_x = uav_position.x - obs_position.x;
    double delta_dist_ob_y = uav_position.y - obs_position.y;
    double delta_dist_ob_z = uav_position.z - obs_position.z;

    my_obs_dist = sqrt(pow(delta_dist_ob_x,2) + pow(delta_dist_ob_y,2)) - 0.25;

//    double time_now = (double) ros::Time::now().sec + ((double) ros::Time::now().nsec / (double) 1E9);
//    f_data_recorder<<time_now<<"\t"<<num_episode_<<"\t"<<goal_reach<<"\t"<<
//                             uav_position.x<<"\t"<<uav_position.y<<"\t"<<
//                          uav_position.z<<"\t"<<goal_X<<"\t"<<
//                             goal_Y<<"\t"<<goal_Z<<"\t"<<
//                             uav_linear_vel.x<<"\t"<<uav_linear_vel.y<<"\t"<<uav_linear_vel.z<<std::endl;

    if(dynamic_env == true)
     {
       if((uav_position.x > -1.4 && uav_position.x < -1.36) || (uav_position.x > -0.3 && uav_position.x < -0.25))
       {
           gazebo_msgs::SetModelState model_msg_obstacle_cylinder;
           model_msg_obstacle_cylinder.request.model_state.model_name = "moving_obstacle_squared1";
           model_msg_obstacle_cylinder.request.model_state.pose.position.x = uav_position.x + 1.2;
           model_msg_obstacle_cylinder.request.model_state.pose.position.y = uav_position.y;
           model_msg_obstacle_cylinder.request.model_state.pose.position.z = 0.0;
           if (gazebo_set_model_state_srv_.call(model_msg_obstacle_cylinder))
           {

           }
           else{
               ROS_ERROR("ENV_INFO: Failed to call set model state");
           }
       }
    }


    srand (time(NULL));

    if((reach_sub_goal == true && has_target == false) /*|| collision_ == true*/)
    {

        double lower_lim_y_init = random_low_lim_y_initial;
        double upper_lim_y_init = random_up_lim_y_initial;
        double rand_pos_y_init = lower_lim_y_init +
                static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(upper_lim_y_init-lower_lim_y_init)));
        double lower_lim_y_goal = random_low_lim_y;
        double upper_lim_y_goal = random_up_lim_y;

        double rand_pos_y_goal = lower_lim_y_goal +
                static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(upper_lim_y_goal-lower_lim_y_goal)));
        gazebo_msgs::SetModelState uav_pose_reset;
        uav_pose_reset.request.model_state.model_name = "firefly1";
        uav_pose_reset.request.model_state.pose.position.x = random_initial_x;
        uav_pose_reset.request.model_state.pose.position.y = rand_pos_y_init;
        uav_pose_reset.request.model_state.pose.position.z = 1.0;
        if (gazebo_set_model_state_srv_.call(uav_pose_reset))
        {

        }
        else{
            ROS_ERROR("ENV_INFO: Failed to call set model state");
        }


//        geometry_msgs::PoseStamped pose_reset_p;
//        pose_reset_p.pose.position.x = random_initial_x;
//        pose_reset_p.pose.position.y = rand_pos_y_init;
//        pose_reset_p.pose.position.z = 1.0;


//        pose_reset.publish(pose_reset_p);

        gazebo_msgs::SetModelState goal_pose_reset;
        goal_pose_reset.request.model_state.model_name = "goal";
        goal_pose_reset.request.model_state.pose.position.x = random_goal_x;
        goal_pose_reset.request.model_state.pose.position.y = rand_pos_y_goal;
        goal_pose_reset.request.model_state.pose.position.z = 0.0;
        if (gazebo_set_model_state_srv_.call(goal_pose_reset))
        {

        }
        else{
            ROS_ERROR("ENV_INFO: Failed to call set model state");
        }

        target_pt.x() = random_goal_x;
        target_pt.y() = rand_pos_y_goal;
        target_pt.z() = 1.2;


        if(has_target = false)
        {
            has_target = true;
           // reach_sub_goal = false;
        }

//        if(collision_ = true)
//        {
//           collision_ = false;
//        }
       // num_episode_++;

    }


}

