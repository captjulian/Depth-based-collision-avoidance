#include "fast_replan_using_sdf.h"


Fast_Replan_Using_SDF::Fast_Replan_Using_SDF()
{


}


Fast_Replan_Using_SDF::~Fast_Replan_Using_SDF()
{

}


void Fast_Replan_Using_SDF::init()
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
    if(random_goal_x == -100)
        random_goal_x = 50;
    ros::param::get("~random_up_lim_y", random_up_lim_y);
    if(random_up_lim_y == -100)
        random_up_lim_y = 35;
    ros::param::get("~random_low_lim_y_left", random_up_lim_y_left);
    if(random_up_lim_y_left == -100)
        random_up_lim_y_left = 35;
    ros::param::get("~random_up_lim_y_right", random_up_lim_y_right);
    if(random_up_lim_y_right == -100)
        random_up_lim_y_right = 35;
    ros::param::get("~random_up_lim_x_up", random_up_lim_x_up);
    if(random_up_lim_x_up == -100)
        random_up_lim_x_up = 35;
    ros::param::get("~random_low_lim_x_down", random_up_lim_x_down);
    if(random_up_lim_x_down == -100)
        random_up_lim_x_down = 35;
    ros::param::get("~random_up_lim_y", random_up_lim_y);
    if(random_up_lim_y == -100)
        random_up_lim_y = 35;
    ros::param::get("~random_low_lim_y", random_low_lim_y);
    if(random_low_lim_y == -100)
        random_low_lim_y = 15;
    ros::param::get("~random_up_lim_y_initial", random_up_lim_y_initial);
    if(random_up_lim_y_initial == -100)
        random_up_lim_y_initial = 35;
    ros::param::get("~random_low_lim_y_initial", random_low_lim_y_initial);
    if(random_low_lim_y_initial == -100)
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


    has_odom = false;
    has_target = true;
    reach_sub_goal = false;
    receive_msg = false;
    close_to_obstacle = 0;

    _trajGene = new TrajectoryGeneratorWaypoint();
    _dev_order = 3;

//    start_pt.x() = -3;
//    start_pt.y() = 0;
//    start_pt.z() = altitude;
    start_vel(0) = 0;
    start_vel(1) = 0;
    start_vel(2) = 0;

    num_episode_ = 1;
    goal_reach = 0;

    my_yaw.setX(0);
    my_yaw.setY(0);
    my_yaw.setZ(0);
    my_yaw.setW(1);

    f_data_recorder.open(file_name);
//    f_data_recorder<<"time"<<"\t"<<"num_episode"<<"\t"<<"goal_reached"<<"\t"<<"collision_status"<<"\t"<<"uav_posx"<<"\t"<<"uav_posy"<<"\t"<<"uav_posz"<<"\t"<<
//                         "target_posx"<<"\t"<<"target_posy"<<"\t"<<"target_posz"<<"\t"<<
//                         "speed_x"<<"\t"<<"speed_y"<<"\t"<<"speed_z"<<"\t"<<"moving_obs_x"<<"\t"<<"moving_obs_y"<<"\t"<<"moving_obs_z"<<std::endl;

    f_data_recorder<<"time"<<"\t"<<"num_episode"<<"\t"<<"goal_reached"<<"\t"<<"collision_status"<<"\t"<<"uav_posx"<<"\t"<<"uav_posy"<<"\t"<<"uav_posz"<<"\t"<<
                         "target_posx"<<"\t"<<"target_posy"<<"\t"<<"target_posz"<<"\t"<<
                         "speed_x"<<"\t"<<"speed_y"<<"\t"<<"speed_z"<<std::endl;

//    f_data_recorder<<"time"<<std::endl;
}


void Fast_Replan_Using_SDF::open(ros::NodeHandle n)
{
    init();
    sdf_cd.init();
    //sub
    scan_sub = n.subscribe("/depth_transfor/scan", 1, &Fast_Replan_Using_SDF::Depth_to_ScanCallback, this);
   // scan_sub = n.subscribe("/drone7/scan", 1, &Depth_BASED_RRT_STAR::Depth_to_ScanCallback, this);
    drone_pos_subs_ = n.subscribe("/firefly1/odometry_sensor1/odometry",1,
                                  &Fast_Replan_Using_SDF::dronePosCallback,this);
    uav_pose_velocity_subs_ = n.subscribe("/gazebo/model_states", 10, &Fast_Replan_Using_SDF::PoseVelocityCallback, this);
    //pub
    rrt_path_pub = n.advertise<visualization_msgs::Marker>("rrt_path",1,true);
    traj_control_pub = n.advertise<trajectory_msgs::MultiDOFJointTrajectory>("/firefly1/command/trajectory", 1);
    tra_visual_ = n.advertise<visualization_msgs::Marker>("trajectory",1,true);
    pose_reset = n.advertise<geometry_msgs::PoseStamped>("/firefly1/command/pose", 1, true);

    //timer
     _exec_timer = n.createTimer(ros::Duration(0.01), &Fast_Replan_Using_SDF::execCallback, this);

    //service
     gazebo_set_model_state_srv_ = n.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
}

void Fast_Replan_Using_SDF::run()
{

}

// change the state to the new state
void Fast_Replan_Using_SDF::changeState(STATE new_state, string pos_call) {
  string state_str[5] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "EXEC_TRAJ",
                         "REPLAN_TRAJ"};
  int pre_s = int(exec_state);
  exec_state = new_state;
  cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " +
              state_str[int(new_state)]
       << endl;
}

void Fast_Replan_Using_SDF::printState() {
  string state_str[5] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "EXEC_TRAJ",
                         "REPLAN_TRAJ"};
  cout << "[Clock]: state: " + state_str[int(exec_state)] << endl;
}

void Fast_Replan_Using_SDF::execCallback(const ros::TimerEvent &e)
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

      bool success = trajGeneration();

      if (success)
        changeState(EXEC_TRAJ, "STATE");
      else
        changeState(GEN_NEW_TRAJ, "STATE");
      break;
    }

    case EXEC_TRAJ: {
      ros::Time time_now = ros::Time::now();
      double t_cur = (time_now - time_traj_start).toSec();
      double t_replan = ros::Duration(1, 0).toSec();
      t_cur = min(time_duration, t_cur);
      odom_pt.x() = drone_odom.pose.pose.position.x;
      odom_pt.y() = drone_odom.pose.pose.position.y;
      odom_pt.z() = drone_odom.pose.pose.position.z;

      //checking collision
      std::pair<Eigen::Vector3d, double> cur_pt = getTrajPos(odom_pt, t_cur);
      std::pair<Eigen::Vector3d, double> next_pt = getTrajPos(cur_pt.first, cur_pt.second);
      std::pair<Eigen::Vector3d, double> next_pt1 = getTrajPos(next_pt.first, next_pt.second);
//      std::pair<Eigen::Vector3d, double> next_pt2 = getTrajPos(next_pt1.first, next_pt1.second);
//      std::pair<Eigen::Vector3d, double> next_pt3 = getTrajPos(next_pt2.first, next_pt2.second);

//     std::cout << "current_pt :" << " " << cur_pt.first.x() << " "
//                << cur_pt.first.y() << " " << cur_pt.first.z() << std::endl;
//     std::cout << "next_pt :" << " " << next_pt.first.x() << " "
//                << next_pt.first.y() << " " << next_pt.first.z() << std::endl;
//     std::cout << "next_pt1 :" << " " << next_pt1.first.x() << " "
//                << next_pt1.first.y() << " " << next_pt1.first.z() << std::endl;
//     std::cout << "next_pt2 :" << " " << next_pt2.first.x() << " "
//                << next_pt2.first.y() << " " << next_pt2.first.z() << std::endl;
//     std::cout << "next_pt3 :" << " " << next_pt3.first.x() << " "
//                << next_pt3.first.y() << " " << next_pt3.first.z() << std::endl;
      bool collision0 = sdf_cd.checkCollision_pt(cur_pt.first, drone_odom);
      bool collision1 = sdf_cd.checkCollision_pt(next_pt.first, drone_odom);
      bool collision2 = sdf_cd.checkCollision_pt(next_pt1.first, drone_odom);
//      bool collision3 = sdf_cd.checkCollision_pt(next_pt2.first, drone_odom);
//      bool collision4 = sdf_cd.checkCollision_pt(next_pt3.first, drone_odom);
      if(collision0 == true || collision1 == true || collision2 == true /*|| collision3 == true || collision4 == true*/)
      {
        std::cout << "inside_collision !" <<" "<< std::endl;
        collision0 = false;
        collision1 = false;
        collision2 = false;
//        collision3 = false;
//        collision4 = false;
        changeState(REPLAN_TRAJ, "STATE");

      }


    if ((target_pt - odom_pt).norm() < no_replan_thresh) {
      return;
    } else if ((start_pt - odom_pt).norm() < replan_thresh) {
      return;
    } else if (t_cur < t_replan) {
      return;}
    else{
//        changeState(REPLAN_TRAJ, "STATE");
    }

      break;
    }
    case REPLAN_TRAJ: {
      ros::Time time_now = ros::Time::now();
      double t_cur = (time_now - time_traj_start).toSec();
      double t_delta = ros::Duration(0, 50).toSec();
      t_cur = t_delta + t_cur;

      start_pt = getPos(t_cur);
      start_vel = getVel(t_cur);

//      clock_t startTime,endTime;
//      startTime = clock();
      bool success = trajGeneration();
//      endTime = clock();
//      double t_dur = (double)(endTime - startTime)/ CLOCKS_PER_SEC;
//      std::cout << "planning time:"<< t_dur <<std::endl;
      if (success)
        changeState(EXEC_TRAJ, "STATE");
      else
        changeState(GEN_NEW_TRAJ, "STATE");



      break;
    }
    }
}

void Fast_Replan_Using_SDF::dronePosCallback(const nav_msgs::Odometry &msg)
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
    drone_odom.pose.pose.orientation.x = msg.pose.pose.orientation.x;
    drone_odom.pose.pose.orientation.y = msg.pose.pose.orientation.y;
    drone_odom.pose.pose.orientation.z = msg.pose.pose.orientation.z;

    has_odom                           = true;
}

void Fast_Replan_Using_SDF::Depth_to_ScanCallback(const sensor_msgs::LaserScan &msg)
{

    sdf_cd.scan_info_.laser_ranges_.clear();
    sdf_cd.scan_info_.laser_ranges_.insert(sdf_cd.scan_info_.laser_ranges_.end(), msg.ranges.begin(), msg.ranges.end());

    //std::cout << "num_range="<<scan_info_.laser_ranges_.size()<<std::endl;
    min_range_t = *std::min_element(sdf_cd.scan_info_.laser_ranges_.begin(), sdf_cd.scan_info_.laser_ranges_.end());

//    clock_t startTime,endTime;
//    startTime = clock();
    sdf_cd.SdffromScan();
//    endTime = clock();
//    double t_dur = (double)(endTime - startTime)/ CLOCKS_PER_SEC;
//    std::cout << "mapping time:"<< t_dur <<std::endl;
//    f_data_recorder<<t_dur<<std::endl;
//    geometry_msgs::Pose drone_pose;
//    drone_pose.position.x = drone_odom.pose.pose.position.x;
//    drone_pose.position.y = drone_odom.pose.pose.position.y;
//    drone_pose.position.z = drone_odom.pose.pose.position.z;
//    drone_pose.orientation.x = drone_odom.pose.pose.orientation.x;
//    drone_pose.orientation.y = drone_odom.pose.pose.orientation.y;
//    drone_pose.orientation.z = drone_odom.pose.pose.orientation.z;
//    drone_pose.orientation.w = drone_odom.pose.pose.orientation.w;
//    double dist_obs = sdf_cd.Get_Dist_fromSDF(drone_pose.position.x + 1, drone_pose.position.y + 1, drone_pose);

//    std::cout << "dist_to_obs ==" << dist_obs << std::endl;
    receive_msg = true;
}

vector<Node *> Fast_Replan_Using_SDF::path_short(vector<Node *> rrt_path, nav_msgs::Odometry drone_odom_now)
{
    vector<Node *> rrt_path_rebuild;

    for(int rrt_num = 0; rrt_num < rrt_path.size(); rrt_num++)
    {
        rrt_path_rebuild.push_back(rrt_path[rrt_path.size() -1 -rrt_num]);

//        std::cout << "path_point ==" << rrt_path[rrt_path.size() -1 -rrt_num]->position.x() << " "
//                  << rrt_path[rrt_path.size() -1 -rrt_num]->position.y() << std::endl;
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

        //std::cout << "X1 ==" << X1 <<std::endl;
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

//            Node* node1;
//            node1 = new Node;
//            node1->position.x() = end0.x();
//            node1->position.y() = end0.y();
//            short_cut_path.push_back(node1);

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

//            std::cout << "path_point ==" << short_cut_path[j]->position.x() << " "
//                      << short_cut_path[j]->position.y() << std::endl;

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

vector<Node *> Fast_Replan_Using_SDF::rrt_star_path_search(nav_msgs::Odometry drone_odom_now, geometry_msgs::Pose end_point)
{

    RRTSTAR rrtstar_now;
    rrtstar_now.setMaxIterations(100);
    rrtstar_now.setStepSize(1.0);
    rrtstar_now.setSTART(drone_odom_now.pose.pose.position.x,drone_odom_now.pose.pose.position.y);
    rrtstar_now.setEND(end_point.position.x,end_point.position.y);
    srand (time(NULL));
    int iter = 0;

    rrtstar_now.path.clear();

//    std::cout << "start_x ="<< " "<< rrtstar.startPos.x() << "start_y ="<< " "<<rrtstar.startPos.y() << std::endl;
//    std::cout << "end_x ="<< " "<< rrtstar.endPos.x() << "end_y ="<< " "<<rrtstar.endPos.y() << std::endl;
    if(end_point.position.x > 1)
    {
        Xmin = -1;
        Xmax = end_point.position.x;
    }
    else if(end_point.position.x < -1)
    {
        Xmin = end_point.position.x;
        Xmax = 1;
    }
    else
    {
        Xmin = -2.5;
        Xmax = 2.5;
    }

    if(end_point.position.y > 1)
    {
        Ymin = -1;
        Ymax = end_point.position.y;
    }
    else if(end_point.position.y < -1)
    {
        Ymin = end_point.position.y;
        Ymax = 1;
    }
    else
    {
        Ymin = -2.5;
        Ymax = 2.5;
    }
     // only_go_forward//-1.5
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
//        std::cout << "iter =="<< " " <<iter<<std::endl;

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

double Fast_Replan_Using_SDF::GetRandomReal(int low, int up)
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

Eigen::VectorXd Fast_Replan_Using_SDF::timeAllocation(Eigen::MatrixXd Path)
{
    Eigen::VectorXd time(Path.rows() - 1);

    int num = Path.rows() - 1;

    for(int i = 0; i < num; i++)
    {
        geometry_msgs::Point p_0;
        p_0.x = Path(i, 0);
        p_0.y = Path(i, 1);
        p_0.z = Path(i, 2);

        geometry_msgs::Point p_1;
        p_1.x = Path(i+1, 0);
        p_1.y = Path(i+1, 1);
        p_1.z = Path(i+1, 2);

        double delta_x = p_1.x - p_0.x;
        double delta_y = p_1.y - p_0.y;
        double delta_z = p_1.z - p_0.z;
        double path_length = sqrt(pow(delta_x, 2) + pow(delta_y, 2) + pow(delta_z, 2));

        double t1 = _Vel/_Acc;


        double t2 = path_length/_Vel - t1;

        time(i) = floor((2*t1 + t2)*100)/100;
    }
   // std::cout << time.rows() << std::endl;

    return time;
}

bool Fast_Replan_Using_SDF::trajGeneration()
{

          end_pos.position.x = target_pt.x();
          end_pos.position.y = target_pt.y();
          end_pos.position.z = target_pt.z();

//          if (_polyCoeff.rows() > 0)
//          {
//              drone_odom.pose.pose.position.x =  start_pt.x();

//              drone_odom.pose.pose.position.y =  start_pt.y();

//              drone_odom.pose.pose.position.z =  start_pt.z();
//          }

//          start_vel(0) = 0;
//          start_vel(1) = 0;
//          start_vel(2) = 0;
          vector<Node *> short_path;
          vector<Node *> path = rrt_star_path_search(drone_odom,end_pos);
          //vector<Node *> short_path = path_short(path, drone_odom);

            for(int rrt_num = 0; rrt_num < path.size(); rrt_num++)
            {
                short_path.push_back(path[path.size() -1 -rrt_num]);

                std::cout << "path_point ==" << path[path.size() -1 -rrt_num]->position.x() << " "
                          << path[path.size() -1 -rrt_num]->position.y() << std::endl;
            }

          if(short_path.size() > 0)
          {

              publish_rrt_path(short_path);
              Eigen::MatrixXd path_traj(int(short_path.size()), 3);
              for (int k = 0; k < int(short_path.size()); k++) {
                path_traj(k,0) = floor(short_path[k]->position.x()*100)/100;
                path_traj(k,1) = floor(short_path[k]->position.y()*100)/100;
                path_traj(k,2) = altitude;
//                std::cout << "path_p = " << " " << path_traj(k,0) << " " << path_traj(k,1)
//                          << " " << path_traj(k,2) << std::endl;

           }

              //traj_opt
              trajOptimization(path_traj);
              time_duration = _polyTime.sum();

              // Publish the trajectory
              trajPublish(_polyCoeff, _polyTime);

              time_traj_start = ros::Time::now();
              // return if the trajectory generation successes
              if (_polyCoeff.rows() > 0)
                return true;
              else
                return false;
          }

}

void Fast_Replan_Using_SDF::trajOptimization(Eigen::MatrixXd path)
{


    Eigen::MatrixXd vel = Eigen::MatrixXd::Zero(2, 3);
    Eigen::MatrixXd acc = Eigen::MatrixXd::Zero(2, 3);


    vel.row(0) = start_vel;

    /**
     *
     *finish the timeAllocation() using resonable allocation
     *
     * **/
    _polyTime = timeAllocation(path);

//    std::cout << "time " << " " << _polyTime << std::endl;

    /**
     *
     * generate a minimum-jerk piecewise monomial polynomial-based
     * trajectory
     *
     * **/
    _polyCoeff =
        _trajGene->PolyQPGeneration(_dev_order, path, vel, acc, _polyTime);

    //vis_traj


    if(std::isfinite(_polyCoeff(0,0)))
        visTrajectory(_polyCoeff,_polyTime);
    else
        std::cout << "trajectory generation fail !!" << std::endl;


}





Eigen::Vector3d Fast_Replan_Using_SDF::getPos(double t_cur)
{
    double time = 0;
    Eigen::Vector3d pos = Eigen::Vector3d::Zero();
    for (int i = 0; i < _polyTime.size(); i++) {
      for (double t = 0.0; t < _polyTime(i); t += 0.01) {
        time = time + 0.01;
        if (time > t_cur) {
          pos = _trajGene->getPosPoly(_polyCoeff, i, t);
          return pos;
        }
      }
    }
    return pos;
}

std::pair<Eigen::Vector3d, double> Fast_Replan_Using_SDF::getTrajPos(Eigen::Vector3d odom, double time)
{
    Eigen::Vector3d pos = Eigen::Vector3d::Zero();

    std::pair<Eigen::Vector3d, double> out;
    out.first = pos;
    out.second = 0;

    double t_sum;

    for (int i = 0; i < _polyTime.size(); i++) {
      for (double t = 0.0; t < _polyTime(i); t += 0.01) {

        pos = _trajGene->getPosPoly(_polyCoeff, i, t);

        //std::cout << "pos ==" << pos.x() << " " << pos.y() << std::endl;

        double delta_x = odom.x() - pos.x();
        double delta_y = odom.y() - pos.y();
       // double delta_z = odom.z() - pos.z();

        double dist = sqrt(pow(delta_x, 2) + pow(delta_y, 2));

        if(i == 0)
        {
            t_sum = t;
        }
        else
        {
            double t_pre = 0;
            int k = i;
            while(k > 0)
            {
               k = k - 1;
               t_pre = t_pre + _polyTime(k);
            }
            t_sum = t + t_pre;
        }
        if (dist > 0.25 && t_sum > time) {
          out.first = pos;
          out.second = t_sum;
          return out;
        }
      }
    }
    return out;

}


Eigen::Vector3d Fast_Replan_Using_SDF::getVel(double t_cur) {
  double time = 0;
  Eigen::Vector3d Vel = Eigen::Vector3d::Zero();
  for (int i = 0; i < _polyTime.size(); i++) {
    for (double t = 0.0; t < _polyTime(i); t += 0.01) {
      time = time + 0.01;
      if (time > t_cur) {
        Vel = _trajGene->getVelPoly(_polyCoeff, i, t);
        return Vel;
      }
    }
  }
  return Vel;
}

void Fast_Replan_Using_SDF::publish_rrt_path(vector<Node *> path_)
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

void Fast_Replan_Using_SDF::visTrajectory(Eigen::MatrixXd polyCoeff, Eigen::VectorXd time)
{
    visualization_msgs::Marker _traj_vis;

    _traj_vis.header.stamp = ros::Time::now();
    _traj_vis.header.frame_id = "/world";

    _traj_vis.ns = "trajectory";
    _traj_vis.id = 0;
    _traj_vis.type = visualization_msgs::Marker::SPHERE_LIST;
    _traj_vis.action = visualization_msgs::Marker::ADD;
    _traj_vis.scale.x = 0.15;
    _traj_vis.scale.y = 0.15;
    _traj_vis.scale.z = 0.15;
    _traj_vis.pose.orientation.x = 0.0;
    _traj_vis.pose.orientation.y = 0.0;
    _traj_vis.pose.orientation.z = 0.0;
    _traj_vis.pose.orientation.w = 1.0;

    _traj_vis.color.a = 1.0;
    _traj_vis.color.r = 0.0;
    _traj_vis.color.g = 0.5;
    _traj_vis.color.b = 1.0;

    _traj_vis.points.clear();
    Vector3d pos;
    geometry_msgs::Point pt;

    for (int i = 0; i < time.size(); i++) {
      for (double t = 0.0; t < time(i); t += 0.01) {
        pos = _trajGene->getPosPoly(polyCoeff, i, t);
        pt.x = pos(0);
        pt.y = pos(1);
        pt.z = pos(2);
//        std::cout << "pt == " << " " << pt.x << " " << pt.y << " " << pt.z << std::endl;
        _traj_vis.points.push_back(pt);
      }
    }
    tra_visual_.publish(_traj_vis);
}

void Fast_Replan_Using_SDF::trajPublish(Eigen::MatrixXd polyCoeff, Eigen::VectorXd time)
{
    if (polyCoeff.size() == 0 || time.size() == 0) {
      ROS_WARN("[trajectory_generator_waypoint] empty trajectory, nothing to "
               "publish.");
      return;
    }

    my_traj.points.clear();
    Eigen::Vector3d pos, vel, acc;

    int path_num = 0;
    for (int i = 0; i < time.size(); i++) {
      for (double t = 0.0; t < time(i); t += 0.01) {
        pos = _trajGene->getPosPoly(polyCoeff, i, t);
        vel = _trajGene->getVelPoly(polyCoeff, i, t);
        acc = _trajGene->getAccPoly(polyCoeff, i, t);
        trajectory_msgs::MultiDOFJointTrajectoryPoint my_point;
        geometry_msgs::Transform position_vector;

        position_vector.translation.x = pos(0);
        position_vector.translation.y = pos(1);
        position_vector.translation.z = pos(2);
        position_vector.rotation.x = my_yaw.getX();
        position_vector.rotation.y = my_yaw.getY();
        position_vector.rotation.z = my_yaw.getZ();
        position_vector.rotation.w = my_yaw.getW();

        my_point.transforms.push_back(position_vector);

        geometry_msgs::Twist vel_vector;

        vel_vector.linear.x = vel(0);
        vel_vector.linear.y = vel(1);
        vel_vector.linear.z = vel(2);

    //            vel_vector.angular.x = states[j].angular_velocity_W[0];
    //            vel_vector.angular.y = states[j].angular_velocity_W[1];
    //            vel_vector.angular.z = states[j].angular_velocity_W[2];

        my_point.velocities.push_back(vel_vector);

        geometry_msgs::Twist acc_vector;

        acc_vector.linear.x = acc(0);
        acc_vector.linear.y = acc(1);
        acc_vector.linear.z = acc(2);

    //            acc_vector.angular.x = states[j].angular_acceleration_W[0];
    //            acc_vector.angular.y = states[j].angular_acceleration_W[1];
    //            acc_vector.angular.z = states[j].angular_acceleration_W[2];

        my_point.accelerations.push_back(acc_vector);

        my_traj.points.push_back(my_point);


        my_traj.header.frame_id   = std::string("/world");
        my_traj.header.stamp = ros::Time::now();
        if(i == 0)
        {
            my_traj.points[path_num].time_from_start = ros::Duration(t);
        }
        else
        {
            double t_pre = 0;
            int k = i;
            while(k > 0)
            {
               k = k - 1;
               t_pre = t_pre + time(k);
            }
            my_traj.points[path_num].time_from_start = ros::Duration(t + t_pre);
        }

        path_num++;

      }
    }
  traj_control_pub.publish(my_traj);


}

void Fast_Replan_Using_SDF::PoseVelocityCallback(const gazebo_msgs::ModelStates::ConstPtr &msg)
{

    cv::Point3f uav_position, uav_linear_vel;
    cv::Point3f obs1_position, obs2_position,obs3_position,obs4_position, obs5_position, obs6_position;
    cv::Point3f obs7_position, obs8_position,obs9_position,obs10_position, obs11_position, obs12_position;
    cv::Point3f obs13_position, obs14_position,obs15_position,obs16_position, obs17_position, obs18_position;
    cv::Point3f obs19_position, obs20_position,obs21_position,obs22_position, obs23_position, obs24_position;
    for (int i=0; i<msg->name.size(); i++)
    {
        if (msg->name[i].compare("firefly1") == 0)
        {

            uav_position.x = msg->pose[i].position.x;
            uav_position.y = msg->pose[i].position.y;
            uav_position.z = msg->pose[i].position.z;

            uav_linear_vel.x = msg->twist[i].linear.x;
            uav_linear_vel.y = msg->twist[i].linear.y;
            uav_linear_vel.z = msg->twist[i].linear.z;
            break;
        }
//        if(msg->name[i].compare("moving_obstacle_cylinder1") == 0)
//        {
//            obs1_position.x = msg->pose[i].position.x;
//            obs1_position.y = msg->pose[i].position.y;
//            obs1_position.z = msg->pose[i].position.z;
//        }
//        if(msg->name[i].compare("moving_obstacle_squared1") == 0)
//        {
//            obs2_position.x = msg->pose[i].position.x;
//            obs2_position.y = msg->pose[i].position.y;
//            obs2_position.z = msg->pose[i].position.z;
//        }
            if(msg->name[i].compare("obstacle_squared1") == 0)
            {
                obs1_position.x = msg->pose[i].position.x;
                obs1_position.y = msg->pose[i].position.y;
                obs1_position.z = msg->pose[i].position.z;
            }
            if(msg->name[i].compare("obstacle_squared2") == 0)
            {
                obs2_position.x = msg->pose[i].position.x;
                obs2_position.y = msg->pose[i].position.y;
                obs2_position.z = msg->pose[i].position.z;
            }
           if(msg->name[i].compare("obstacle_squared3") == 0)
           {
               obs3_position.x = msg->pose[i].position.x;
               obs3_position.y = msg->pose[i].position.y;
               obs3_position.z = msg->pose[i].position.z;
           }
           if(msg->name[i].compare("obstacle_squared4") == 0)
           {
               obs4_position.x = msg->pose[i].position.x;
               obs4_position.y = msg->pose[i].position.y;
               obs4_position.z = msg->pose[i].position.z;
           }
           if(msg->name[i].compare("obstacle_squared5") == 0)
           {
               obs5_position.x = msg->pose[i].position.x;
               obs5_position.y = msg->pose[i].position.y;
               obs5_position.z = msg->pose[i].position.z;
           }
           if(msg->name[i].compare("obstacle_squared6") == 0)
           {
               obs6_position.x = msg->pose[i].position.x;
               obs6_position.y = msg->pose[i].position.y;
               obs6_position.z = msg->pose[i].position.z;
           }

           if(msg->name[i].compare("obstacle_squared7") == 0)
           {
               obs7_position.x = msg->pose[i].position.x;
               obs7_position.y = msg->pose[i].position.y;
               obs7_position.z = msg->pose[i].position.z;
           }
           if(msg->name[i].compare("obstacle_squared8") == 0)
           {
               obs8_position.x = msg->pose[i].position.x;
               obs8_position.y = msg->pose[i].position.y;
               obs8_position.z = msg->pose[i].position.z;
           }
          if(msg->name[i].compare("obstacle_squared9") == 0)
          {
              obs9_position.x = msg->pose[i].position.x;
              obs9_position.y = msg->pose[i].position.y;
              obs9_position.z = msg->pose[i].position.z;
          }
          if(msg->name[i].compare("obstacle_squared10") == 0)
          {
              obs10_position.x = msg->pose[i].position.x;
              obs10_position.y = msg->pose[i].position.y;
              obs10_position.z = msg->pose[i].position.z;
          }
          if(msg->name[i].compare("obstacle_squared11") == 0)
          {
              obs11_position.x = msg->pose[i].position.x;
              obs11_position.y = msg->pose[i].position.y;
              obs11_position.z = msg->pose[i].position.z;
          }
          if(msg->name[i].compare("obstacle_squared12") == 0)
          {
              obs12_position.x = msg->pose[i].position.x;
              obs12_position.y = msg->pose[i].position.y;
              obs12_position.z = msg->pose[i].position.z;
          }

          if(msg->name[i].compare("obstacle_squared13") == 0)
          {
              obs13_position.x = msg->pose[i].position.x;
              obs13_position.y = msg->pose[i].position.y;
              obs13_position.z = msg->pose[i].position.z;
          }
          if(msg->name[i].compare("obstacle_squared14") == 0)
          {
              obs14_position.x = msg->pose[i].position.x;
              obs14_position.y = msg->pose[i].position.y;
              obs14_position.z = msg->pose[i].position.z;
          }
         if(msg->name[i].compare("obstacle_squared15") == 0)
         {
             obs15_position.x = msg->pose[i].position.x;
             obs15_position.y = msg->pose[i].position.y;
             obs15_position.z = msg->pose[i].position.z;
         }
         if(msg->name[i].compare("obstacle_squared16") == 0)
         {
             obs16_position.x = msg->pose[i].position.x;
             obs16_position.y = msg->pose[i].position.y;
             obs16_position.z = msg->pose[i].position.z;
         }
         if(msg->name[i].compare("obstacle_squared17") == 0)
         {
             obs17_position.x = msg->pose[i].position.x;
             obs17_position.y = msg->pose[i].position.y;
             obs17_position.z = msg->pose[i].position.z;
         }
         if(msg->name[i].compare("obstacle_squared18") == 0)
         {
             obs18_position.x = msg->pose[i].position.x;
             obs18_position.y = msg->pose[i].position.y;
             obs18_position.z = msg->pose[i].position.z;
         }

         if(msg->name[i].compare("obstacle_squared19") == 0)
         {
             obs19_position.x = msg->pose[i].position.x;
             obs19_position.y = msg->pose[i].position.y;
             obs19_position.z = msg->pose[i].position.z;
         }
         if(msg->name[i].compare("obstacle_squared20") == 0)
         {
             obs20_position.x = msg->pose[i].position.x;
             obs20_position.y = msg->pose[i].position.y;
             obs20_position.z = msg->pose[i].position.z;
         }
        if(msg->name[i].compare("obstacle_squared21") == 0)
        {
            obs21_position.x = msg->pose[i].position.x;
            obs21_position.y = msg->pose[i].position.y;
            obs21_position.z = msg->pose[i].position.z;
        }
        if(msg->name[i].compare("obstacle_squared22") == 0)
        {
            obs22_position.x = msg->pose[i].position.x;
            obs22_position.y = msg->pose[i].position.y;
            obs22_position.z = msg->pose[i].position.z;
        }
        if(msg->name[i].compare("obstacle_squared23") == 0)
        {
            obs23_position.x = msg->pose[i].position.x;
            obs23_position.y = msg->pose[i].position.y;
            obs23_position.z = msg->pose[i].position.z;
        }
        if(msg->name[i].compare("obstacle_squared24") == 0)
        {
            obs24_position.x = msg->pose[i].position.x;
            obs24_position.y = msg->pose[i].position.y;
            obs24_position.z = msg->pose[i].position.z;
        }

    }


//    if(dynamic_env == true)
//     {
//       if((uav_position.x > -1.4 && uav_position.x < -1.36) || (uav_position.x > -0.3 && uav_position.x < -0.25))
//       {
//           gazebo_msgs::SetModelState model_msg_obstacle_cylinder;
//           model_msg_obstacle_cylinder.request.model_state.model_name = "moving_obstacle_squared1";
//           model_msg_obstacle_cylinder.request.model_state.pose.position.x = uav_position.x + 1.5;
//           model_msg_obstacle_cylinder.request.model_state.pose.position.y = uav_position.y;
//           model_msg_obstacle_cylinder.request.model_state.pose.position.z = 0.0;
//           if (gazebo_set_model_state_srv_.call(model_msg_obstacle_cylinder))
//           {

//           }
//           else{
//               ROS_ERROR("ENV_INFO: Failed to call set model state");
//           }
//       }
//    }

    //distance to obstacle
    double delta_x_obs1 = uav_position.x - obs1_position.x;
    double delta_y_obs1 = uav_position.y - obs1_position.y;
    double dist_to_obs1 = sqrt(pow(delta_x_obs1,2) + pow(delta_y_obs1, 2));

    double delta_x_obs2 = uav_position.x - obs2_position.x;
    double delta_y_obs2 = uav_position.y - obs2_position.y;
    double dist_to_obs2 = sqrt(pow(delta_x_obs2,2) + pow(delta_y_obs2, 2));

    double delta_x_obs3 = uav_position.x - obs3_position.x;
    double delta_y_obs3 = uav_position.y - obs3_position.y;
    double dist_to_obs3 = sqrt(pow(delta_x_obs3,2) + pow(delta_y_obs3, 2));

    double delta_x_obs4 = uav_position.x - obs4_position.x;
    double delta_y_obs4 = uav_position.y - obs4_position.y;
    double dist_to_obs4 = sqrt(pow(delta_x_obs4,2) + pow(delta_y_obs4, 2));

    double delta_x_obs5 = uav_position.x - obs5_position.x;
    double delta_y_obs5 = uav_position.y - obs5_position.y;
    double dist_to_obs5 = sqrt(pow(delta_x_obs5,2) + pow(delta_y_obs5, 2));

    double delta_x_obs6 = uav_position.x - obs6_position.x;
    double delta_y_obs6 = uav_position.y - obs6_position.y;
    double dist_to_obs6 = sqrt(pow(delta_x_obs6,2) + pow(delta_y_obs6, 2));

    double delta_x_obs7 = uav_position.x - obs7_position.x;
    double delta_y_obs7 = uav_position.y - obs7_position.y;
    double dist_to_obs7 = sqrt(pow(delta_x_obs7,2) + pow(delta_y_obs7, 2));

    double delta_x_obs8 = uav_position.x - obs8_position.x;
    double delta_y_obs8 = uav_position.y - obs8_position.y;
    double dist_to_obs8 = sqrt(pow(delta_x_obs8,2) + pow(delta_y_obs8, 2));

    double delta_x_obs9 = uav_position.x - obs9_position.x;
    double delta_y_obs9 = uav_position.y - obs9_position.y;
    double dist_to_obs9 = sqrt(pow(delta_x_obs9,2) + pow(delta_y_obs9, 2));

    double delta_x_obs10 = uav_position.x - obs10_position.x;
    double delta_y_obs10 = uav_position.y - obs10_position.y;
    double dist_to_obs10 = sqrt(pow(delta_x_obs10,2) + pow(delta_y_obs10, 2));

    double delta_x_obs11 = uav_position.x - obs11_position.x;
    double delta_y_obs11 = uav_position.y - obs11_position.y;
    double dist_to_obs11 = sqrt(pow(delta_x_obs11,2) + pow(delta_y_obs11, 2));

    double delta_x_obs12 = uav_position.x - obs12_position.x;
    double delta_y_obs12 = uav_position.y - obs12_position.y;
    double dist_to_obs12 = sqrt(pow(delta_x_obs12,2) + pow(delta_y_obs12, 2));

    double delta_x_obs13 = uav_position.x - obs13_position.x;
    double delta_y_obs13 = uav_position.y - obs13_position.y;
    double dist_to_obs13 = sqrt(pow(delta_x_obs13,2) + pow(delta_y_obs13, 2));

    double delta_x_obs14 = uav_position.x - obs14_position.x;
    double delta_y_obs14 = uav_position.y - obs14_position.y;
    double dist_to_obs14 = sqrt(pow(delta_x_obs14,2) + pow(delta_y_obs14, 2));

    double delta_x_obs15 = uav_position.x - obs15_position.x;
    double delta_y_obs15 = uav_position.y - obs15_position.y;
    double dist_to_obs15 = sqrt(pow(delta_x_obs15,2) + pow(delta_y_obs15, 2));

    double delta_x_obs16 = uav_position.x - obs16_position.x;
    double delta_y_obs16 = uav_position.y - obs16_position.y;
    double dist_to_obs16 = sqrt(pow(delta_x_obs16,2) + pow(delta_y_obs16, 2));

    double delta_x_obs17 = uav_position.x - obs17_position.x;
    double delta_y_obs17 = uav_position.y - obs17_position.y;
    double dist_to_obs17 = sqrt(pow(delta_x_obs17,2) + pow(delta_y_obs17, 2));

    double delta_x_obs18 = uav_position.x - obs18_position.x;
    double delta_y_obs18 = uav_position.y - obs18_position.y;
    double dist_to_obs18 = sqrt(pow(delta_x_obs18,2) + pow(delta_y_obs18, 2));

    double delta_x_obs19 = uav_position.x - obs19_position.x;
    double delta_y_obs19 = uav_position.y - obs19_position.y;
    double dist_to_obs19 = sqrt(pow(delta_x_obs19,2) + pow(delta_y_obs19, 2));

    double delta_x_obs20 = uav_position.x - obs20_position.x;
    double delta_y_obs20 = uav_position.y - obs20_position.y;
    double dist_to_obs20 = sqrt(pow(delta_x_obs20,2) + pow(delta_y_obs20, 2));

    double delta_x_obs21 = uav_position.x - obs21_position.x;
    double delta_y_obs21 = uav_position.y - obs21_position.y;
    double dist_to_obs21 = sqrt(pow(delta_x_obs21,2) + pow(delta_y_obs21, 2));

    double delta_x_obs22 = uav_position.x - obs22_position.x;
    double delta_y_obs22 = uav_position.y - obs22_position.y;
    double dist_to_obs22 = sqrt(pow(delta_x_obs22,2) + pow(delta_y_obs22, 2));

    double delta_x_obs23 = uav_position.x - obs23_position.x;
    double delta_y_obs23 = uav_position.y - obs23_position.y;
    double dist_to_obs23 = sqrt(pow(delta_x_obs23,2) + pow(delta_y_obs23, 2));

    double delta_x_obs24 = uav_position.x - obs24_position.x;
    double delta_y_obs24 = uav_position.y - obs24_position.y;
    double dist_to_obs24 = sqrt(pow(delta_x_obs24,2) + pow(delta_y_obs24, 2));

    if(dist_to_obs1 < 0.25 || dist_to_obs2 < 0.25 || dist_to_obs3 < 0.25 || dist_to_obs4 < 0.25 || dist_to_obs5 < 0.25 || dist_to_obs6 < 0.25
           || dist_to_obs7 < 0.25 || dist_to_obs8 < 0.25 || dist_to_obs9 < 0.25 || dist_to_obs10 < 0.25 || dist_to_obs11 < 0.25 || dist_to_obs12 < 0.25
            || dist_to_obs13 < 0.25 || dist_to_obs14 < 0.25 || dist_to_obs15 < 0.25 || dist_to_obs16 < 0.25 || dist_to_obs17 < 0.25 || dist_to_obs18 < 0.25
            || dist_to_obs19 < 0.25 || dist_to_obs20 < 0.25 || dist_to_obs21 < 0.25 || dist_to_obs22 < 0.25 || dist_to_obs23 < 0.25 || dist_to_obs24 < 0.25)
    {
//       std::cout << "close_to_obstacle !!" << std::endl;
       close_to_obstacle = 1;
    }

    //------------------------------------------------------
    double delta_x_goal = uav_position.x - target_pt.x();
    double delta_y_goal = uav_position.y - target_pt.y();
   // double delta_z_goal = uav_position.z - target_pt.z();
    double dist_to_goal = sqrt(pow(delta_x_goal,2) + pow(delta_y_goal, 2));

    if(dist_to_goal < 0.02)
    {
        std::cout << "coming inside reset !" << std::endl;
        reach_sub_goal = true;
        has_target = false;
        goal_reach = 1;
    }

    double time_now = (double) ros::Time::now().sec + ((double) ros::Time::now().nsec / (double) 1E9);
//    f_data_recorder<<time_now<<"\t"<<num_episode_<<"\t"<<goal_reach<<"\t"<<close_to_obstacle<<"\t"<<
//                             uav_position.x<<"\t"<<uav_position.y<<"\t"<<
//                          uav_position.z<<"\t"<<target_pt.x()<<"\t"<<
//                             target_pt.y()<<"\t"<<target_pt.z()<<"\t"<<
//                             uav_linear_vel.x<<"\t"<<uav_linear_vel.y<<"\t"<<uav_linear_vel.z<<"\t"<<obs2_position.x<<"\t"<<obs2_position.y<<
//                            "\t"<<obs2_position.z<<std::endl;

    f_data_recorder<<time_now<<"\t"<<num_episode_<<"\t"<<goal_reach<<"\t"<<close_to_obstacle<<"\t"<<
                             uav_position.x<<"\t"<<uav_position.y<<"\t"<<
                          uav_position.z<<"\t"<<target_pt.x()<<"\t"<<
                             target_pt.y()<<"\t"<<target_pt.z()<<"\t"<<
                             uav_linear_vel.x<<"\t"<<uav_linear_vel.y<<"\t"<<uav_linear_vel.z<<std::endl;
    srand (time(NULL));

    double rand_pos_y_goal;

    if((reach_sub_goal == true && has_target == false) /*|| collision_ == true*/)
    {
        int num = (rand()%4)+ 1;
        std::cout << "num ==" << num << std::endl;
        if (num == 1)
        {
            double upper_lim_y_goal = random_up_lim_y_left;
            double lower_lim_y_goal = random_up_lim_y_right;

            double upper_lim_x_goal = random_up_lim_x_down +1;
            double lower_lim_x_goal = random_up_lim_x_down;

            rand_pos_y_goal = lower_lim_y_goal +
                    static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(upper_lim_y_goal-lower_lim_y_goal)));

            random_goal_x = lower_lim_x_goal +
                    static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(upper_lim_x_goal-lower_lim_x_goal)));

        }
        else if (num == 2)
        {
            double upper_lim_y_goal = random_up_lim_y_left;
            double lower_lim_y_goal = random_up_lim_y_left - 1;

            double upper_lim_x_goal = random_up_lim_x_up;
            double lower_lim_x_goal = random_up_lim_x_down;

            rand_pos_y_goal = lower_lim_y_goal +
                    static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(upper_lim_y_goal-lower_lim_y_goal)));

            random_goal_x = lower_lim_x_goal +
                    static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(upper_lim_x_goal-lower_lim_x_goal)));
        }
        else if (num == 3)
        {
            double upper_lim_y_goal = random_up_lim_y_left;
            double lower_lim_y_goal = random_up_lim_y_right;

            double upper_lim_x_goal = random_up_lim_x_up;
            double lower_lim_x_goal = random_up_lim_x_up - 1;

            rand_pos_y_goal = lower_lim_y_goal +
                    static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(upper_lim_y_goal-lower_lim_y_goal)));

            random_goal_x = lower_lim_x_goal +
                    static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(upper_lim_x_goal-lower_lim_x_goal)));
        }
        else
        {
            double upper_lim_y_goal = random_up_lim_y_right + 1;
            double lower_lim_y_goal = random_up_lim_y_right;

            double upper_lim_x_goal = random_up_lim_x_up;
            double lower_lim_x_goal = random_up_lim_x_down;

            rand_pos_y_goal = lower_lim_y_goal +
                    static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(upper_lim_y_goal-lower_lim_y_goal)));

            random_goal_x = lower_lim_x_goal +
                    static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(upper_lim_x_goal-lower_lim_x_goal)));
        }
        std::cout << "coming inside reset true!" << std::endl;

        double lower_lim_y_init = random_low_lim_y_initial;
        double upper_lim_y_init = random_up_lim_y_initial;
        double rand_pos_y_init = lower_lim_y_init +
                static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(upper_lim_y_init-lower_lim_y_init)));
//        double lower_lim_y_goal = random_low_lim_y;
//        double upper_lim_y_goal = random_up_lim_y;

//        double rand_pos_y_goal = lower_lim_y_goal +
//                static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(upper_lim_y_goal-lower_lim_y_goal)));
//        gazebo_msgs::SetModelState uav_pose_reset;

//        double yaw = atan2(rand_pos_y_goal, random_goal_x);
        gazebo_msgs::SetModelState uav_pose_reset;

        double yaw = atan2(rand_pos_y_goal, random_goal_x);

        if (rand_pos_y_goal == 0&& random_goal_x > 0)
           {
              yaw = 3.14/2;
           }
        if (rand_pos_y_goal == 0&& random_goal_x < 0)
           {
              yaw = -3.14/2;
           }
         //std::cout << "yaw" << yaw << std::endl;
         tf::Quaternion quaternion = tf::createQuaternionFromRPY(0,0,yaw);

         my_yaw = quaternion;

        uav_pose_reset.request.model_state.model_name = "firefly1";
        uav_pose_reset.request.model_state.pose.position.x = random_initial_x;
        uav_pose_reset.request.model_state.pose.position.y = rand_pos_y_init;
        uav_pose_reset.request.model_state.pose.position.z = 1.0;
        uav_pose_reset.request.model_state.pose.orientation.x = quaternion.getX();
        uav_pose_reset.request.model_state.pose.orientation.y = quaternion.getY();
        uav_pose_reset.request.model_state.pose.orientation.z = quaternion.getZ();
        uav_pose_reset.request.model_state.pose.orientation.w = quaternion.getW();
        if (gazebo_set_model_state_srv_.call(uav_pose_reset))
        {

        }
        else{
            ROS_ERROR("ENV_INFO: Failed to call set model state");
        }


        geometry_msgs::PoseStamped pose_reset_p;
        pose_reset_p.pose.position.x = random_initial_x;
        pose_reset_p.pose.position.y = rand_pos_y_init;
        pose_reset_p.pose.position.z = 1.2;
        pose_reset_p.pose.orientation.x = quaternion.getX();
        pose_reset_p.pose.orientation.y = quaternion.getY();
        pose_reset_p.pose.orientation.z = quaternion.getZ();
        pose_reset_p.pose.orientation.w = quaternion.getW();

        pose_reset.publish(pose_reset_p);

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


        if(has_target == false)
        {
            has_target = true;
            reach_sub_goal = false;
            close_to_obstacle = 0;
            goal_reach = 0;
        }
        sleep(1);
        changeState(INIT, "STATE");

//        if(collision_ = true)
//        {
//           collision_ = false;
//        }
        num_episode_++;
    }



}


