clc;close all;clear all;
%{'time    num_episode goal_reached    uav_posx    uav_posy    target_posx target_posy moving_obstacle_posx   moving_obstacle_posy   speed_x speed_y min_distance_to_obstacles'}
cell_str = importdata('/home/liang/workspace/ros/aerostack_catkin_ws/src/aerostack_stack/stack_devel/Depth_Based_RRT_STAR/launch/document.txt');

num_total_episodes = max(cell_str.data(:,2));

num_good_episodes_vec = [];
for i=1:num_total_episodes
    ind = find(cell_str.data(:,2) == i);
    if(length(ind) < 2) % NOT a valid episode
        continue;
    else
        num_good_episodes_vec = [num_good_episodes_vec i];
    end
end

num_good_episodes = length(num_good_episodes_vec);
num_good_episodes_vec_rand = num_good_episodes_vec(randperm(length(num_good_episodes_vec)));

num_times_goal_is_reached = 0;

distance_to_closest_obstacle = [];
average_velocity_per_episode = [];
maximum_velocity_per_episode = [];
velocity_per_episode         = [];
distance_per_episode         = [];
time_to_reach_the_goal       = [];
NUM_EPISODES_FOR_METRICS     = 300;

for i=1:NUM_EPISODES_FOR_METRICS
    ind = find(cell_str.data(:,2) == num_good_episodes_vec_rand(i));

    total_distance = 0.0;
    total_velocity = 0.0;
    for j=2:length(ind)
        delta_x = cell_str.data(ind(j),5) - cell_str.data(ind(j-1),5);
        delta_y = cell_str.data(ind(j),6) - cell_str.data(ind(j-1),6);
        delta_distance = sqrt(delta_x^2 + delta_y^2);
        total_distance = total_distance + delta_distance;
        
        delta_v_x = cell_str.data(ind(j),11);
        delta_v_y = cell_str.data(ind(j),12);
        velocity  = sqrt(delta_v_x^2 + delta_v_y^2);
        velocity_per_episode = [velocity_per_episode velocity];
    end

    
    if(sum(cell_str.data(ind,3)) == 1 && sum(cell_str.data(ind,4)) == 0)
       num_times_goal_is_reached = num_times_goal_is_reached + 1;
       distance_per_episode = [distance_per_episode total_distance];
       time = cell_str.data(ind(end),1) - cell_str.data(ind(1),1);
       time_to_reach_the_goal = [time_to_reach_the_goal time];
       mean_velocity = mean(velocity_per_episode);
       maximum_velocity = max(velocity_per_episode);
       average_velocity_per_episode = [average_velocity_per_episode mean_velocity];
       maximum_velocity_per_episode = [maximum_velocity_per_episode maximum_velocity];
    end
end

performance = (num_times_goal_is_reached/NUM_EPISODES_FOR_METRICS)*100

mean_distance_per_episode = mean(distance_per_episode)
std_distance_per_episode = std(distance_per_episode)

mean_time_to_reach_the_goal = mean(time_to_reach_the_goal)
std_time_to_reach_the_goal = std(time_to_reach_the_goal)

mean_average_velocity = mean(average_velocity_per_episode)
std_average_velocity = std(average_velocity_per_episode)

mean_max_velocity = mean(maximum_velocity_per_episode)
std_max_velocity = std(maximum_velocity_per_episode)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% PLOT EXAMPLES 2D TRAJECTORIES
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
T_gazebo_matlab = [0 -1 2.2;1 0 4;0 0 1];
wall_color_3D = 'b';
wall_color = 'b';
wall_width = 2;

rectangle('position',[0, 0, 0.1, 8.0], 'FaceColor', wall_color)
rectangle('position',[0.1, 8.0, 4.4, 0.1], 'FaceColor', wall_color)
rectangle('position',[4.4, 0, 0.1, 8.0], 'FaceColor', wall_color)
rectangle('position',[0.1, 0, 4.4, 0.1], 'FaceColor', wall_color)
xlabel('X (m)','FontSize',22,'FontWeight','bold')
ylabel('Y (m)','FontSize',22,'FontWeight','bold')
axis equal;
grid on;
hold on
obstacle_color = [0.2 0.9 0];

obstacle_width = 0.5;
pos_obs_gazebo_1 = [1.5 1.5 1] + [-obstacle_width/2.0 obstacle_width/2.0 0];
pos_obs_gazebo_2 = [1.5 -1.5 1] + [-obstacle_width/2.0 obstacle_width/2.0 0];
pos_obs_gazebo_3 = [-2.0 0.0 1] + [-0.3 0.3 0];
pos_obs_gazebo_4 = [-2.0 0.45 1] + [-0.3 0.3 0];
pos_obs_gazebo_5 = [-2.0 -0.45 1] + [-0.3 0.3 0];
pos_obs_gazebo_6 = [-2.0 -0.25 1] + [-0.3 0.3 0];
pos_obs_gazebo_7 = [-2.0 0.25 1] + [-0.3 0.3 0];

pos_obs_1 = T_gazebo_matlab*pos_obs_gazebo_1';
pos_obs_2 = T_gazebo_matlab*pos_obs_gazebo_2';
pos_obs_3 = T_gazebo_matlab*pos_obs_gazebo_3';
pos_obs_4 = T_gazebo_matlab*pos_obs_gazebo_4';
pos_obs_5 = T_gazebo_matlab*pos_obs_gazebo_5';
pos_obs_6 = T_gazebo_matlab*pos_obs_gazebo_6';
pos_obs_7 = T_gazebo_matlab*pos_obs_gazebo_7';
%Draw static rectnagle obstacles of ENV4
rectangle('position',[pos_obs_1(1), pos_obs_1(2),obstacle_width, obstacle_width],'LineWidth',2,'FaceColor', obstacle_color)
rectangle('position',[pos_obs_2(1), pos_obs_2(2), obstacle_width, obstacle_width],'LineWidth',2','FaceColor', obstacle_color)
rectangle('position',[pos_obs_3(1), pos_obs_3(2), 0.6, 0.6], 'Curvature', [1 1] , 'LineStyle','--', 'EdgeColor', [0.7 0.2 0], 'LineWidth', 2)
rectangle('position',[pos_obs_4(1), pos_obs_4(2), 0.6, 0.6], 'Curvature', [1 1] , 'LineStyle','--', 'EdgeColor', [0.7 0.2 0], 'LineWidth', 2)
rectangle('position',[pos_obs_5(1), pos_obs_5(2), 0.6, 0.6], 'Curvature', [1 1] , 'LineStyle','--', 'EdgeColor', [0.7 0.2 0], 'LineWidth', 2)
rectangle('position',[pos_obs_6(1), pos_obs_6(2), 0.6, 0.6], 'Curvature', [1 1] , 'LineStyle','--', 'EdgeColor', [0.7 0.2 0], 'LineWidth', 2)
rectangle('position',[pos_obs_7(1), pos_obs_7(2), 0.6, 0.6], 'Curvature', [1 1] , 'LineStyle','--', 'EdgeColor', [0.7 0.2 0], 'LineWidth', 2)

hold on;
ind_plot_trajetory = find(cell_str.data(:,2) == num_good_episodes_vec_rand(1));
pos_x = cell_str.data(ind_plot_trajetory,5);
pos_y = cell_str.data(ind_plot_trajetory,6);
obstacle_pos_x = cell_str.data(ind_plot_trajetory,14);
obstacle_pos_y = cell_str.data(ind_plot_trajetory,15);
trajectory = zeros(length(pos_x), 2);
for i=1:length(pos_x)
    hold on;
    pos = T_gazebo_matlab*[pos_x(i) pos_y(i) 1]';
    trajectory(i,1) = pos(1);
    trajectory(i,2) = pos(2);
    
    if i > 1
        if((obstacle_pos_x(i) == obstacle_pos_x(i-1) && (obstacle_pos_y(i) == obstacle_pos_y(i-1))))
            continue
        end
        obstacle_pos = [obstacle_pos_x(i) obstacle_pos_y(i) 1] + [-0.3 0.3 0];
        pos_obstacle = T_gazebo_matlab*obstacle_pos';
        rectangle('position',[pos_obstacle(1), pos_obstacle(2), 0.6, 0.6],'LineWidth',2,'LineStyle','--','EdgeColor', [0.7 0.2 0])
    end
end
hold on;
plot(trajectory(:,1), trajectory(:,2))
hold on;
plot(trajectory(1,1), trajectory(1,2),'+','MarkerEdgeColor','r','MarkerSize',10,'LineWidth',2)
l_tra =length(trajectory);
plot(trajectory(l_tra,1), trajectory(l_tra,2),'*','MarkerEdgeColor','r','MarkerSize',10,'LineWidth',2)
% obstacle_pos_x = cell_str.data(ind_plot_trajetory,8);
% obstacle_pos_y = cell_str.data(ind_plot_trajetory,9);
% trajectory = zeros(length(obstacle_pos_x), 2);
% for i=1:length(obstacle_pos_x)
%     pos = T_gazebo_matlab*[obstacle_pos_x(i) obstacle_pos_y(i) 1]';
%     rectangle('position',[pos(1), pos(2), 0.6, 0.6],'LineWidth',2,'LineStyle','--','EdgeColor', [0.7 0.2 0])
% end

% xlim([0,11]);
% ylim([0,12]);


