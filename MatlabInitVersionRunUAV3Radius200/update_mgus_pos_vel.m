%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Description %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author  : Baolin Yin
% Data    : 05.05, 2023
% Email   : 932261247@qq.com
% Version : V1.0
% Function: Updating the position and velocity of the MGUs and the center.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Description %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Input %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% vel_mgus_old  :The velocity of MGUs before updating. Shape:(num_mgus, 2) Unit: m/s
% vel_old_center:The velocity of center before updating. Shape:(1, 2) Unit: m/s
% pos_mgus_old  :The position of MGUs before updating. Shape:(num_mgus, 2) Unit: m
% pos_center_old:The position of center before updating. Shape:(1, 2) Unit: m
% time_interval :The time interval. Unit:s
% num_mgus      :The number of MGUs.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Input %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Output %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% vel_new_mgus  :The velocity of MGUs after updating. Shape:(num_mgus, 2) Unit: m/s
% vel_new_center:The velocity of center after updating. Shape:(1, 2) Unit: m/s
% pos_new       :The position of MGUs after updating. Shape:(num_mgus, 2) Unit: m
% pos_new_center:The position of center after updating. Shape:(1, 2) Unit: m
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Output %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [vel_new_mgus,vel_new_center,pos_mgus_new,pos_new_center] = update_mgus_pos_vel(vel_mgus_old,vel_old_center,pos_mgus_old,pos_center_old,time_interval,num_mgus)
pos_mgus_new = pos_mgus_old + (vel_mgus_old + reshape(vel_old_center,[1,1,2])) * time_interval;
pos_new_center = pos_center_old + reshape(vel_old_center,[1,2]) * time_interval;
vel_new_mgus = normrnd(3, 3, [num_mgus, 2]);  % 修改参数别忘了改generate中的参数
vel_new_center = normrnd(1, 3, [1, 2]);
end

