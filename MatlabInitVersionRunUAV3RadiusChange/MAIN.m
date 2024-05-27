%%
clc;
clear;
close all;
% 1. Initialize the environment and parameters.
time_interval = 0.5; % Time interval(the length of the time slot). Unit:s
center_init = [0,0]; % The init center of MGUs group. Unit:m
num_mgu = 2000; % The number of MGUs.
radius_range = 800; % The range of the MGUs' distribution. Unit:m
time_data_init = 1800; % The init time used to colect position of MGUs for forcasting. Unit:s
num_slot_data_init = time_data_init / time_interval; % The number of time slot that generate trajectory of MGUs.
velocity_mgus = zeros(num_slot_data_init, num_mgu, 2); % The velocity of  MGUs.
velocity_center = zeros(num_slot_data_init,2); % The velocity of MGUs' center.
position_mgu = zeros(num_slot_data_init, num_mgu, 2); % The position of MGUs.
position_center = zeros(num_slot_data_init,2); % The position of center.
num_uav = 3; % The number of UAVs.
num_subc_uav = 10; % The number pf suncarriers of each UAV.
num_agent = num_uav * num_subc_uav; % The number of agent. 
speed_uav = 40; % The speed of UAV.
gap_height = 500; % The flight height gap of UAV.
height_max = 4000; % The maximum flight height of UAV.
height_uav = [height_max, height_max - gap_height, height_max - 2 * gap_height]; % The flight height of each UAV.
beta_0 = 10 .^ (-50 / 10); % The channel gain when distance is 1m.
bw_all = 30;
bw_subc = bw_all / (num_subc_uav * num_subc_uav); % The bandwidth of each subcarrier. MHz
noise_den = 10 .^ ((-220 - 30) / 10) * (10 .^ 6); % The noise density. W/MHz
noise_sub = noise_den * bw_subc; % The noise power of each subcarrier. W
p_uav_max = 10 .^ ((30 - 30) / 10);  % 50dBm;
max_episodes = 200; % The maximum episodes in RL train.
num_end_slot = 20;
thet_uav = pi ./ 3;

% 2. Generating the MGUs. Return init position and velocity.
[velocity_mgus(1,:,:),velocity_center(1,:),position_mgu(1,:,:),position_center(1,:)] ...
    = generate_mgus(center_init,num_mgu,radius_range);
% 3.Generating the trajectory of MGUs.
for index_data = 2:num_slot_data_init
    [velocity_mgus(index_data,:,:),velocity_center(index_data,:),position_mgu(index_data,:,:),position_center(index_data,:)] ...
        = update_mgus_pos_vel(velocity_mgus(index_data-1,:,:),velocity_center(index_data-1,:),position_mgu(index_data-1,:,:),...
        position_center(index_data-1,:),time_interval,num_mgu);
end
% Trajectory of UAVs at current round.
pos_mgu_init = reshape(position_mgu(1,:,:),[],2);
[radius_init_mgu,center_init_uav_mgu, ~,  res_cluster_init] = cluster_area_overlap_descent(pos_mgu_init,num_uav,num_mgu);
tra_radius_init = calculate_radius_uavs(radius_init_mgu,height_uav, thet_uav);
time_slot_one_cir_init = (ceil((2 * pi * tra_radius_init) / speed_uav)) / time_interval;
time_slot_two_cir_init = 2 * time_slot_one_cir_init;
time_start = min(time_slot_one_cir_init);
time_end = max(time_slot_two_cir_init);

pos_all = position_mgu(time_start:time_end,:,:);
pos_average = zeros(num_mgu,2);
for index_slot=1:(time_end - time_start)
    pos_average = pos_average + reshape(pos_all(index_slot,:,:),[],2);
end
pos_average = pos_average / (time_end - time_start);
pos_mgu_final = pos_average;
[radius_final_mgu,center_final_uav_mgu, ~,  ~] = cluster_area_overlap_descent(pos_mgu_final,num_uav,num_mgu);
tra_radius_final = calculate_radius_uavs(radius_final_mgu,height_uav, thet_uav);

figure(2);
axis equal;
scatter(pos_mgu_init(:,1), pos_mgu_init(:,2),20,'b','filled');
hold on;
scatter(pos_mgu_final(:,1), pos_mgu_final(:,2),20,'g','filled');
hold on;


theta = linspace(0, 2 * pi, 1000);
for cluster_index = 1:num_uav
    x_start = center_init_uav_mgu(cluster_index, 1) + tra_radius_init(cluster_index) * cos(theta);
    y_start = center_init_uav_mgu(cluster_index, 2) + tra_radius_init(cluster_index) * sin(theta);
    if cluster_index == 1
        plot(x_start, y_start,'r',LineWidth=1.5);
    elseif cluster_index == 2
        plot(x_start, y_start,'m',LineWidth=1.5);
    else
        plot(x_start, y_start,'k',LineWidth=1.5);
    end
end


hold on;
for cluster_index = 1:num_uav
    x_start = center_final_uav_mgu(cluster_index, 1) + tra_radius_final(cluster_index) * cos(theta);
    y_start = center_final_uav_mgu(cluster_index, 2) + tra_radius_final(cluster_index) * sin(theta);
    if cluster_index == 1
        plot(x_start, y_start,'r--',LineWidth=1.5);
    elseif cluster_index == 2
        plot(x_start, y_start,'m--',LineWidth=1.5);
    else
        plot(x_start, y_start,'k--',LineWidth=1.5);
    end
end
hold on;
final_point = zeros(num_uav, 2);
k = zeros(1,num_uav);
bias = zeros(1,num_uav);
start_uav_init = zeros(num_uav, 2);
for index_uav = 1:num_uav
    [final_point(index_uav,:)] = calculate_final_point_current_cir(center_init_uav_mgu(index_uav,:),center_final_uav_mgu(index_uav,:),tra_radius_init(index_uav),tra_radius_final(index_uav));
    k(index_uav) = -1 / ((final_point(index_uav,2) - center_init_uav_mgu(index_uav,2)) / (final_point(index_uav,1) - center_init_uav_mgu(index_uav,1)));
    bias(index_uav) = final_point(index_uav,2) - k(index_uav) * final_point(index_uav,1);
    [res] = calculate_point_line_cir(center_final_uav_mgu(index_uav,:),tra_radius_final(index_uav),k(index_uav),bias(index_uav));
    x = final_point(index_uav,1):3:res(1);
    if index_uav == 1
        plot(x,k(index_uav) * x + bias(index_uav),'-.r',LineWidth=1.5);
        scatter(final_point(index_uav,1),final_point(index_uav,2),'r','filled');
        hold on;
        start_uav_init(index_uav,:) = [center_init_uav_mgu(index_uav,1) - tra_radius_init(index_uav),center_init_uav_mgu(index_uav,2)];
        scatter(center_init_uav_mgu(index_uav,1) - tra_radius_init(index_uav),center_init_uav_mgu(index_uav,2),'s','r','filled');
    elseif index_uav == 2
        plot(x,k(index_uav) * x + bias(index_uav),'-.m',LineWidth=1.5);
        scatter(final_point(index_uav,1),final_point(index_uav,2),'m','filled');
        hold on;
        start_uav_init(index_uav,:) = [center_init_uav_mgu(index_uav,1) - tra_radius_init(index_uav),center_init_uav_mgu(index_uav,2)];
        scatter(center_init_uav_mgu(index_uav,1) - tra_radius_init(index_uav),center_init_uav_mgu(index_uav,2),'s','m','filled');
    else
        plot(x,k(index_uav) * x + bias(index_uav),'-.k',LineWidth=1.5);
        scatter(final_point(index_uav,1),final_point(index_uav,2),'k','filled');
        hold on;
        start_uav_init(index_uav,:) = [center_init_uav_mgu(index_uav,1) - tra_radius_init(index_uav),center_init_uav_mgu(index_uav,2)];
        scatter(center_init_uav_mgu(index_uav,1) - tra_radius_init(index_uav),center_init_uav_mgu(index_uav,2),'s','k','filled');
    end
    hold on;
end
dis_init = zeros(1,num_uav);
for index_uav = 1:num_uav
    dis_init(index_uav) = 2 * pi * tra_radius_init(index_uav) + calculateArcLength(tra_radius_init(index_uav), center_init_uav_mgu(index_uav,:), ...
        start_uav_init(index_uav,:),final_point(index_uav,:), false);
end

time_all_init = ceil((dis_init / speed_uav) / time_interval);
trajectory_uav1_init = zeros(time_all_init(1),2);
trajectory_uav1_init(1,:) = start_uav_init(1,:);
trajectory_uav2_init = zeros(time_all_init(2),2);
trajectory_uav2_init(1,:) = start_uav_init(2,:);
trajectory_uav3_init = zeros(time_all_init(3),2);
trajectory_uav3_init(1,:) = start_uav_init(3,:);

theta_increase = rad2deg((speed_uav ./ tra_radius_init) * time_interval);
for index_slot = 2:time_all_init(1)
    trajectory_uav1_init(index_slot,:) = calculateTargetPointOnCircle(center_init_uav_mgu(1,1), center_init_uav_mgu(1,2), tra_radius_init(1), trajectory_uav1_init(index_slot-1,1), trajectory_uav1_init(index_slot-1,2), theta_increase(1));
end

for index_slot = 2:time_all_init(2)
    trajectory_uav2_init(index_slot,:) = calculateTargetPointOnCircle(center_init_uav_mgu(2,1), center_init_uav_mgu(2,2), tra_radius_init(2), trajectory_uav2_init(index_slot-1,1), trajectory_uav2_init(index_slot-1,2), theta_increase(2));
end

for index_slot = 2:time_all_init(3)
    trajectory_uav3_init(index_slot,:) = calculateTargetPointOnCircle(center_init_uav_mgu(3,1), center_init_uav_mgu(3,2), tra_radius_init(3), trajectory_uav3_init(index_slot-1,1), trajectory_uav3_init(index_slot-1,2), theta_increase(3));
end
figure(3);
axis equal;
scatter(trajectory_uav1_init(:,1),trajectory_uav1_init(:,2));
hold on;
scatter(trajectory_uav2_init(:,1),trajectory_uav2_init(:,2));
hold on;
scatter(trajectory_uav3_init(:,1),trajectory_uav3_init(:,2));
hold on;
for cluster_index = 1:num_uav
    x_start = center_init_uav_mgu(cluster_index, 1) + tra_radius_init(cluster_index) * cos(theta);
    y_start = center_init_uav_mgu(cluster_index, 2) + tra_radius_init(cluster_index) * sin(theta);
    if cluster_index == 1
        plot(x_start, y_start,'r',LineWidth=1.5);
    elseif cluster_index == 2
        plot(x_start, y_start,'m',LineWidth=1.5);
    else
        plot(x_start, y_start,'k',LineWidth=1.5);
    end
end
hold on;
scatter(final_point(1,1),final_point(1,2),'b','filled');
hold on;
scatter(center_init_uav_mgu(1,1) - tra_radius_init(1),center_init_uav_mgu(1,2),'s','r','filled');

hold on;
scatter(final_point(2,1),final_point(2,2),'b','filled');
hold on;
scatter(center_init_uav_mgu(2,1) - tra_radius_init(2),center_init_uav_mgu(2,2),'s','r','filled');

hold on;
scatter(final_point(3,1),final_point(3,2),'b','filled');
hold on;
scatter(center_init_uav_mgu(3,1) - tra_radius_init(3),center_init_uav_mgu(3,2),'s','r','filled');


%%
% Proposed Scheme
num_slot = max(time_all_init);
C1 = 0;
C2 = 1; 
B1 = -4.3221;
B2 = 6.0750;
a = 3;
rate_vs_episode = struct();
res_rate = zeros(max(time_all_init), num_uav);
pow_des = zeros(max(time_all_init), num_agent);
pow_inf = zeros(max(time_all_init), num_agent);
server_radius_init = tra_radius_init ./ 2;
for index_slot = 33:num_slot
    label = ["slot_" + num2str(index_slot)]; 
    tcp = tcpclient("127.0.0.1",1235);
    pos_mgu_rl = reshape(position_mgu(index_slot,:,:),[ ...
        ],2);
    pos_mgu_uav1 = pos_mgu_rl(res_cluster_init == 1,:);
    pos_mgu_uav2 = pos_mgu_rl(res_cluster_init == 2,:);
    pos_mgu_uav3 = pos_mgu_rl(res_cluster_init == 3,:);
    uav_set_rl = update_uav_set(time_all_init,index_slot,num_uav);
    num_uav_rl = length(uav_set_rl(:));
    pos_uav_rl = zeros(num_uav_rl, 2);
    pos_mgu_server_rl_entry = zeros(num_uav_rl, num_subc_uav, 2);
    gain = zeros(num_uav_rl, num_subc_uav);
    for index_uav = 1:num_uav_rl
        if uav_set_rl(index_uav) == 1
            pos_uav_rl(index_uav, :) = trajectory_uav1_init(index_slot,:);
            dis_uav1_mgu            = sqrt(sum((pos_uav_rl(index_uav, :) - pos_mgu_uav1) .^ 2, 2) + height_uav(index_uav) ^ 2);
            temp1 = sort(dis_uav1_mgu);
            dis_list_serevr_uav1 = temp1(1:num_subc_uav);
            [~,server_mgu_id_uav1] = ismember(dis_uav1_mgu, dis_list_serevr_uav1);
            pos_mgu_server_uav1 = pos_mgu_uav1(server_mgu_id_uav1 ~= 0, :);
            pos_mgu_server_rl_entry(index_uav,:,:) = pos_mgu_server_uav1;
            % gain(index_uav,:) = beta_0 ./ sqrt(sum((pos_uav_rl(index_uav, :) - pos_mgu_server_uav1) .^ 2, 2) + height_uav(index_uav) ^ 2);
            dis_server = reshape(dis_uav1_mgu(server_mgu_id_uav1 ~= 0),1,[]);
            f_ag = C1 + C2 ./ (1 + exp(-(B1 + B2 .* (height_uav(index_uav) ./ dis_server))));
            gain(index_uav,:) = beta_0 .* f_ag .* dis_server .^ (-a);
        elseif uav_set_rl(index_uav) == 2
            pos_uav_rl(index_uav, :) = trajectory_uav2_init(index_slot,:);
            dis_uav2_mgu = sqrt(sum((pos_uav_rl(index_uav, :) - pos_mgu_uav2) .^ 2, 2) + height_uav(index_uav) ^ 2);
            temp2 = sort(dis_uav2_mgu);
            dis_list_serevr_uav2 = temp2(1:num_subc_uav);
            [~,server_mgu_id_uav2] = ismember(dis_uav2_mgu, dis_list_serevr_uav2);
            pos_mgu_server_uav2 = pos_mgu_uav2(server_mgu_id_uav2 ~= 0, :);
            pos_mgu_server_rl_entry(index_uav,:,:) = pos_mgu_server_uav2;
            % gain(index_uav,:) = beta_0 ./ sqrt(sum((pos_uav_rl(index_uav, :) - pos_mgu_server_uav2) .^ 2, 2) + height_uav(index_uav) ^ 2);
            dis_server = reshape(dis_uav2_mgu(server_mgu_id_uav2 ~= 0),1,[]);
            f_ag = C1 + C2 ./ (1 + exp(-(B1 + B2 .* (height_uav(index_uav) ./ dis_server))));
            gain(index_uav,:) = beta_0 .* f_ag .* dis_server .^ (-a);
        else
            pos_uav_rl(index_uav, :) = trajectory_uav3_init(index_slot,:);
            dis_uav3_mgu = sqrt(sum((pos_uav_rl(index_uav, :) - pos_mgu_uav3) .^ 2, 2) + height_uav(index_uav) ^ 2);
            temp3 = sort(dis_uav3_mgu);
            dis_list_serevr_uav3 = temp3(1:num_subc_uav);
            [~,server_mgu_id_uav3] = ismember(dis_uav3_mgu, dis_list_serevr_uav3);
            pos_mgu_server_uav3 = pos_mgu_uav3(server_mgu_id_uav3 ~= 0, :);
            pos_mgu_server_rl_entry(index_uav,:,:) = pos_mgu_server_uav3;
            % gain(index_uav,:) = beta_0 ./ sqrt(sum((pos_uav_rl(index_uav, :) - pos_mgu_server_uav3) .^ 2, 2) + height_uav(index_uav) ^ 2);
            dis_server = reshape(dis_uav3_mgu(server_mgu_id_uav3 ~= 0),1,[]);
            f_ag = C1 + C2 ./ (1 + exp(-(B1 + B2 .* (height_uav(index_uav) ./ dis_server))));
            gain(index_uav,:) = beta_0 .* f_ag .* dis_server .^ (-a);
        end
    end

    pos_uav_rl_entry = zeros(num_uav_rl, 3);
    for index_uav = 1:num_uav_rl
        pos_uav_rl_entry(index_uav,1:2) = pos_uav_rl(index_uav,:);
        pos_uav_rl_entry(index_uav,3) = height_uav(index_uav);
    end
    data.pos_uav_rl_entry = pos_uav_rl_entry;
    data.pos_mgu_server_rl_entry = pos_mgu_server_rl_entry;
    data.gain = gain;
    data.num_subc_uav = num_subc_uav;
    data.bw_subc = bw_subc;
    data.num_agent_rl = num_uav_rl * num_subc_uav;
    data.noise_sub = noise_sub;
    data.p_uav_max = p_uav_max;
    data.max_episodes = max_episodes;
    data.num_uav = num_uav;
    data.num_subc_uav = num_subc_uav;
    data.num_end_slot = num_end_slot;
    jsonStr = jsonencode(data);
    tcp.write(jsonStr);

    while tcp.NumBytesAvailable == 0
        pause(0.1);
    end
    data_read = read(tcp);
    json_data = jsondecode(char(data_read));
    sum_rate_now = reshape(json_data{1,1},1,[]);
    pow_des_now = reshape(json_data{2,1},1,[]);
    pow_inf_now = reshape(json_data{3,1},1,[]);
    sum_rate_vs_episode = reshape(json_data{4,1},1,[]);
    rate_vs_episode.(label) = sum_rate_vs_episode;
    res_rate(index_slot, :) = sum_rate_now;
    pow_des(index_slot,1:num_uav_rl * num_subc_uav) = pow_des_now;
    pow_inf(index_slot,1:num_uav_rl * num_subc_uav) = pow_inf_now;
end
