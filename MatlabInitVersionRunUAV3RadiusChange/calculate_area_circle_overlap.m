%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Description %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author  : Baolin Yin
% Data    : 11,28, 2022
% Email   : 932261247@qq.com
% Version : V1.0
% Function: Calculate overlaping area based on MC.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Description %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Input %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% center       : The center of MGUs cluster. Shape: (M, 2) Unit:m
% radius       : The radius of MGUs cluster. Shape: (M,1) Unit:m
% num_cluster  : The number of cluster/UAV.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Input %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Output %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% area_overlap    : The overlaping area. Unit:m^2
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Output %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [area_overlap] = calculate_area_circle_overlap(center,radius, num_cluster)
% The number of point for MC.
num_point_mc = 5000;
% Calculating the maximum for x and y.
% Calculating the minimum for x and y.
% Getting the rectangular to contain all the MGUs cluster.
x_all_right = center(:, 1) + radius;
y_all_up = center(:, 2) + radius;
x_all_left = center(:, 1) - radius;
y_all_down = center(:, 2) - radius;
x_max = max(x_all_right);
y_max = max(y_all_up);
x_min = min(x_all_left);
y_min = min(y_all_down);
% plot([x_min,x_min],[y_min,y_max], [x_min,x_max],[y_min,y_min], [x_max,x_max],[y_min,y_max], [x_min,x_max],[y_max,y_max]);
% Calculating the area of rectangular.
area_sq = (x_max - x_min) .* (y_max - y_min);
% MC process
x_rand = unifrnd(x_min, x_max, [num_point_mc, 1]);
y_rand = unifrnd(y_min, y_max, [num_point_mc, 1]);
point_rand = [x_rand, y_rand];
% Calculating the distance between each point of MC process to the each cluster center
distance_each_point_to_each_cluster = zeros(num_cluster, num_point_mc);
point_id_in_each_cluster = zeros(num_cluster, num_point_mc);
for cluster_index = 1:num_cluster
    distance_each_point_to_each_cluster(cluster_index,:) = sqrt(sum((point_rand - center(cluster_index,:)) .^ 2, 2));
    point_id_in_each_cluster(cluster_index,:) = (distance_each_point_to_each_cluster(cluster_index,:) <= radius(cluster_index));
end
% Calculating the number of  point in the overlap area.
flag_point = sum(point_id_in_each_cluster, 1);
num_point = sum(flag_point > 1);
% Calcuting the overlaping area.
area_overlap = area_sq .* (num_point ./ num_point_mc);
end

