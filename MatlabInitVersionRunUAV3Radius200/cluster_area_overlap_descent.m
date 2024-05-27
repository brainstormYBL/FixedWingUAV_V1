%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Description %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author  : Baolin Yin
% Data    : 11,28, 2022
% Email   : 932261247@qq.com
% Version : V1.0
% Function: Calculate the best MGUs circule related to distrabution of MGUs and area of overlap.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Description %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Input %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% position_mgus : The position of the MGUs that is predicted. Shape: (N, 2) Unit:m
% num_uav       : The number of UAVs.
% num_mgu       : The number of MGUs.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Input %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Output %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% radius_mgu    : The radius of each MGU circule. Shape: (M, 1) Unit:m
% center_mgu    : The position of each MGUs circule center. Shape: (M,2) Unit:m
% area          : The ovlerlaping area of MGUs circules. Unit:m^2
% res_cluster   : The cluster for each mgus.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Output %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [radius_mgu,center_mgu, area, res_cluster] = cluster_area_overlap_descent(position_mgus,num_uav, num_mgu)
% The maximum interation.
num_episode = 100;
% Storing the overlaping area.
area = zeros(num_episode, 1);
True = 1;
% The init cluster. 
% cluster_id: The cluster ID of each MGUs.
% cluster_center: The cluster center.
[cluster_id,cluster_center] = kmeans(position_mgus,num_uav);
% color = ['b', 'm', 'g', 'd', 'k', 'c'];
% figure(3);
% scatter(cluster_center(:,1), cluster_center(:,2), 'r');
% hold on;
% for mgu_index = 1:num_mgu
%     scatter(position_mgus(mgu_index,1), position_mgus(mgu_index,2), color(cluster_id(mgu_index)));
% end
% hold on; 
% Calculating the init radius of each MGUs circule.
% The radius is the distance between the center and farthest point of the current cluster
radius_each_cluster = zeros(num_uav,1);
for cluster_index = 1:num_uav
    distance = sqrt(sum((position_mgus - cluster_center(cluster_index,:)) .^ 2, 2));
    distance_current_cluster = distance(cluster_id==cluster_index);
    radius_each_cluster(cluster_index) = max(distance_current_cluster);
end
% Plot
% theta = linspace(0, 2 * pi, 1000);
% for cluster_index = 1:num_uav
%     x = cluster_center(cluster_index, 1) + radius_each_cluster(cluster_index) * cos(theta);
%     y = cluster_center(cluster_index, 2) + radius_each_cluster(cluster_index) * sin(theta);
%     plot(x, y);
% end

flag = 1;
done = 0;
% Interation
while True
    % Calculating the overlaping area.
    [area_overlap] = calculate_area_circle_overlap(cluster_center,radius_each_cluster,num_uav);
    area(flag) = area_overlap;
    % Judging whether or not finish.
    % All the centers stop to update, or interation = num_episode
    if flag == num_episode || done == 1
        break;
    end
    % Updating the circular£¬ including the center and radius.
    [cluster_center, radius_each_cluster, done] = update_center_radius_cluster(cluster_center, radius_each_cluster, position_mgus, num_uav, num_mgu, cluster_id);
    flag = flag + 1;
end
radius_mgu = radius_each_cluster;
center_mgu = cluster_center;
% Calculating the distance between center to each MGUs.
dis = zeros(num_uav,num_mgu);
cluster_id_mgus = zeros(num_mgu,1);
for uav_index = 1:num_uav
    dis(uav_index, :) = sqrt(sum((center_mgu(uav_index,:) - position_mgus) .^ 2,2));
end
for mgu_index = 1:num_mgu
    [~, cluster_id_mgus(mgu_index, 1)] = min(dis(:, mgu_index));
end
res_cluster = cluster_id_mgus;
% figure(4);
% len = length(area(area > 0));
% plot(1:1:len, area(area > 0));
% figure(5);
% scatter(center_mgu(:,1), center_mgu(:,2), 'r');
% hold on;
% for mgu_index = 1:num_mgu
%     scatter(position_mgus(mgu_index,1), position_mgus(mgu_index,2), color(res_cluster(mgu_index)));
% end
% hold on;
% theta = linspace(0, 2 * pi, 1000);
% for cluster_index = 1:num_uav
%     x = cluster_center(cluster_index, 1) + radius_mgu(cluster_index) * cos(theta);
%     y = cluster_center(cluster_index, 2) + radius_mgu(cluster_index) * sin(theta);
%     plot(x, y);
% end
end

