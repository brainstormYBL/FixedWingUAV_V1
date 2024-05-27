%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Description %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author  : Baolin Yin
% Data    : 11,28, 2022
% Email   : 932261247@qq.com
% Version : V1.0
% Function: Judging whether or not update center.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Description %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Input %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% center_new    : The center after updating. Shape: (1, 2) Unit:m
% center_old    : The center before updating. Shape: (1, 2) Unit:m
% pos_mgu       : The position of all the MGUs. Shape: (, 2) Unit:m
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Input %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Output %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% update_flag   : Update flag. If value is 1, the uptading the center.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Output %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [update_flag] = judge_whether_or_not_update_center(center_new,center_old,pos_mgu,num_mgus)
% The threshold value of the number of MGUs.
num_epsilon = num_mgus / 4;
% The slope of the line.
slope = -1 ./ ((center_new(2) - center_old(2)) ./ (center_new(1) - center_old(1)));
% The bias of the line.
bias = center_old(2) - slope .* center_old(1);
% I'm going to plug the x-coordinate into the line.
y_pre = slope .* pos_mgu(:, 1) + bias;
% Calculating the number of MGUs above and below the line.
flag_each_up = pos_mgu(:, 2) > y_pre;
num_other_up = sum(flag_each_up);
flag_each_down = pos_mgu(:, 2) <= y_pre;
num_other_down = sum(flag_each_down);
% Judging whether or not update center.
if num_other_up > num_epsilon && num_other_down > num_epsilon
    update_flag = 1;
else
    update_flag = 0;
end
end

