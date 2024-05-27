%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Description %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author  : Baolin Yin
% Data    : 05.05, 2023
% Email   : 932261247@qq.com
% Version : V1.0
% Function: Generating the position and velocity of the MGUs and the center.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Description %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Input %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% center_init :The init center of MGUs. Shape:(1,2) Unit: m
% num_mgus    :The number of MGUs.
% radius_range:The init radius of the range of MGUs.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Input %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Output %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% velocity_mgus  :The velocity of the MGUs. Shape:(num_mgus, 2) Unit:m/s
% velocity_center:The velocityof the center. Shape:(1, 2) Unit:m/s
% position_mgus  :The position of the MGUs. Shape:(num_mgus,2) Unit:m
% position_center:The position of the center. Shape:(1,2) Unit:m
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Output %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [velocity_mgus,velocity_center,position_mgus,position_center] = generate_mgus(center_init,num_mgus,radius_range)
% Generate the MGUs.
position_mgus = zeros(num_mgus, 2);
velocity_mgus = zeros(num_mgus, 2);
theta = deg2rad(rand(num_mgus, 1) * 360);
position_mgus(:, :) = center_init + [(rand(num_mgus, 1) .* radius_range) .* cos(theta), (rand(num_mgus, 1) .* radius_range) .* sin(theta)];
velocity_mgus(:,:) = normrnd(3, 3, [num_mgus, 2]);  % 修改参数别忘了改update中的参数
velocity_center = normrnd(1, 3, [1, 2]);
position_center = center_init;

% figure(1);
% scatter(position_mgus(:, 1), position_mgus(:, 2));
% title("The init distribution of MGUs");
% xlabel('X (m)');
% ylabel('Y (m)');
% hold on;
% scatter(center_init(1), center_init(2));
% plot(radius_range .* cos(deg2rad(linspace(0, 360, 100))), radius_range .* sin(deg2rad(linspace(0, 360, 100))));
% legend('The position of MGUs', 'Center of MGUs', ' Distributed range of MGUs');
end

