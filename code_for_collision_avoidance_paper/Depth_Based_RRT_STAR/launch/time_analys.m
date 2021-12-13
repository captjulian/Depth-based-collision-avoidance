clc;close all;clear all;
%{'time    num_episode goal_reached    uav_posx    uav_posy    target_posx target_posy moving_obstacle_posx   moving_obstacle_posy   speed_x speed_y min_distance_to_obstacles'}
cell_str = importdata('/home/liang/workspace/ros/aerostack_catkin_ws/src/aerostack_stack/stack_devel/Depth_Based_RRT_STAR/launch/document_mapping_time.txt');

v_len = length(cell_str.data);
vector_time = [];
vector_average_time = [];
j = 0;
for i=2:361        
    
    time = cell_str.data(i);
    vector_time = [vector_time time];
    
    if(j == 10)
       average = mean(vector_time);
       vector_average_time = [vector_average_time average];
       vector_time = [];
       j = 0;
    end
    j = j+1;
end
