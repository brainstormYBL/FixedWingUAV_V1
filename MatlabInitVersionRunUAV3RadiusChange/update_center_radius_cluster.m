%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Description %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author  : Baolin Yin
% Data    : 11,28, 2022
% Email   : 932261247@qq.com
% Version : V1.0
% Function: Updating the MGUs cluster,inclding the radius and center.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Description %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Input %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% center_old    : The center before updating. Shape: (M, 2) Unit:m
% radius_old    : The radius before updating. Shape: (M, 1) Unit:m
% pos_mgu       : The position of all the MGUs. Shape: (N, 2) Unit:m
% num_cluster   : The number of cluster/UAV.
% num_mgu       : The number of MGUs.

% cluster_id_x  : The index of the MGUs' cluster.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Input %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Output %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% center_new    : The center after updating. Shape: (M, 2) Unit:m
% radius_new    : The radius after updating. Shape: (M, 1) Unit:m
% done          : End update flag.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Output %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [center_new,radius_new, done] = update_center_radius_cluster(center_old,radius_old, pos_mgu, num_cluster, num_mgu, cluster_id_x)
% The length that update the center. 
length_each_step = 50;
done_flag = 0;
center_new = zeros(num_cluster, 2);
line_each_center_old_to_center_new = [center_old(:, 1) - mean(center_old(:, 1)), center_old(:, 2) - mean(center_old(:, 2))];
angle = atan2(line_each_center_old_to_center_new(:, 2),line_each_center_old_to_center_new(:,1));
[~, cluster_id] = sort(angle);  
% M centers are connected to form an M edge shape
center_sort_old = zeros(num_cluster, 2);
radius_sort_old = zeros(num_cluster, 1);
% figure(2);
for cluster_index = 1:num_cluster
    center_sort_old(cluster_index, :) = center_old(cluster_id(cluster_index), :);
    radius_sort_old(cluster_index) = radius_old(cluster_id(cluster_index));
end
% plot(center_sort_old(:, 1), center_sort_old(:, 2),'r', [center_sort_old(end, 1), center_sort_old(1, 1)], [center_sort_old(end, 2)...
%     , center_sort_old(1, 2)], 'r')
% hold on;
direction_each_center_update = zeros(num_cluster, 2);
% Updating the center
for cluster_index = 1:num_cluster
    if cluster_index == 1
        distance = sqrt(sum((pos_mgu - center_sort_old(1, :)) .^ 2, 2));
        pos_mgu_in = reshape(pos_mgu(distance <= radius_sort_old(1),:),[],2);
        % The old center to the new center
        temp = (center_sort_old(1, :) - center_sort_old(end, :)) + (center_sort_old(1, :) - center_sort_old(2, :));
        % The direction from old center to the new center
        direction_each_center_update(1,:) = temp / norm(temp);
        % The updated length from old center to the new center
        update_length = direction_each_center_update(1,:) .* length_each_step;
        % The center that is updated
        center_new_test = center_sort_old(1, :) + update_length;
        % Judging whether or not to update center
        [update_flag] = judge_whether_or_not_update_center(center_new_test,center_sort_old(1, :),pos_mgu_in,num_mgu);
        if update_flag == 1
            center_new(1,:) = center_new_test;
            center_sort_old(1, :) = center_new_test;
        else
            center_new(1,:) = center_sort_old(1, :);
            done_flag = done_flag + 1;
        end
        % plot([center_sort_old(1,1), center_new(1,1)], [center_sort_old(1,2), center_new(1,2)])
        % hold on;
    elseif cluster_index == num_cluster
        distance = sqrt(sum((pos_mgu - center_sort_old(end, :)) .^ 2, 2));
        pos_mgu_in = reshape(pos_mgu(distance <= radius_sort_old(end),:),[],2);
        temp = center_sort_old(end, :) - center_sort_old(1, :) + center_sort_old(end, :) - center_sort_old(end-1, :);
        direction_each_center_update(end,:) = temp / norm(temp);
        update_length = direction_each_center_update(end,:) .* length_each_step;
        center_new_test = center_sort_old(end, :) + update_length;
        [update_flag] = judge_whether_or_not_update_center(center_new_test,center_sort_old(end, :),pos_mgu_in,num_mgu);
        if update_flag == 1
            center_new(end,:) = center_new_test;
            center_sort_old(end, :) = center_new_test;
        else
            center_new(end,:) = center_sort_old(end, :);
            done_flag = done_flag + 1;
        end
        % plot([center_sort_old(end,1), center_new(end,1)], [center_sort_old(end,2), center_new(end,2)])
        % hold on;
    else
        distance = sqrt(sum((pos_mgu - center_sort_old(cluster_index, :)) .^ 2, 2));
        pos_mgu_in = reshape(pos_mgu(distance <= radius_sort_old(cluster_index),:),[],2);
        temp = center_sort_old(cluster_index, :) - center_sort_old(cluster_index - 1, :) + center_sort_old(cluster_index, :)...
        - center_sort_old(cluster_index + 1, :);
        direction_each_center_update(cluster_index,:) = temp / norm(temp);
        update_length = direction_each_center_update(cluster_index,:) .* length_each_step;
        center_new_test = center_sort_old(cluster_index, :) + update_length;
        [update_flag] = judge_whether_or_not_update_center(center_new_test,center_sort_old(cluster_index, :),pos_mgu_in,num_mgu);
        if update_flag == 1
            center_new(cluster_index,:) = center_new_test;
            center_sort_old(cluster_index, :) = center_new_test;
        else
            center_new(cluster_index,:) = center_sort_old(cluster_index, :);
            done_flag = done_flag + 1;
        end
        % plot([center_sort_old(cluster_index,1), center_new(cluster_index,1)], [center_sort_old(cluster_index,2), center_new(cluster_index,2)])
        % hold on;
    end      
end

% color = ['b', 'm', 'g', 'd', 'k', 'c'];
% figure(3);
% scatter(center_new(:,1), center_new(:,2), 'r');
% hold on;
% for mgu_index = 1:num_mgu
%     scatter(pos_mgu(mgu_index,1), pos_mgu(mgu_index,2), color(cluster_id_x(mgu_index)));
% end
% hold on;
% 
% theta = linspace(0, 2 * pi, 1000);
% for cluster_index = 1:num_cluster
%     x = center_new(cluster_index, 1) + radius_sort_old(cluster_index) * cos(theta);
%     y = center_new(cluster_index, 2) + radius_sort_old(cluster_index) * sin(theta);
%     plot(x, y);
% end

% Updating the radius.
distance_each_mgu_to_center_new = zeros(num_cluster, num_mgu);
mgu_not_in_cri = zeros(num_cluster, num_mgu);
for cluster_index =1:num_cluster
    distance_each_mgu_to_center_new(cluster_index, :) = sqrt(sum((center_new(cluster_index,:) - pos_mgu) .^ 2, 2));
    mgu_not_in_cri(cluster_index, :) = distance_each_mgu_to_center_new(cluster_index, :) > radius_sort_old(cluster_index);
end
temp = sum(mgu_not_in_cri, 1);
mgu_not_in_id = temp == num_cluster;
pos_not_in = reshape(pos_mgu(mgu_not_in_id, :), [], 2);
flag = isempty(pos_not_in);
num_mgu_not_in = sum(mgu_not_in_id);
if flag == 0
    for mgu_not_in_index = 1:num_mgu_not_in
        distance = sqrt(sum((pos_not_in(mgu_not_in_index, :) - center_new(:, :)) .^ 2, 2));
        cluster_id_not_in = distance == min(distance);
        radius_sort_old(cluster_id_not_in) = sqrt(sum((center_new(cluster_id_not_in, :) - pos_not_in(mgu_not_in_index,:)) .^ 2, 2));
    end
end
radius_new = radius_sort_old;
if done_flag == num_cluster
    done = 1;
else
    done = 0;
end
end

