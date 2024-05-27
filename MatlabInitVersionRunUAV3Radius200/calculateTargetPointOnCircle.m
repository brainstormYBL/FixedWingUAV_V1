function target_point = calculateTargetPointOnCircle(x0, y0, r, start_x, start_y, delta_theta)
    % x0, y0: 圆心坐标
    % r: 圆的半径
    % start_x, start_y: 起点坐标
    % delta_theta: 圆心角的值（度）
    
    % 计算起点相对于圆心的极角
    theta1 = atan2(start_y - y0, start_x - x0);
    
    % 将极角转换为弧度
    theta1 = wrapTo2Pi(theta1);
    delta_theta = deg2rad(delta_theta);
    
    % 计算目标点的极角
    theta2 = theta1 + delta_theta;
    
    % 计算目标点的坐标
    x = x0 + r * cos(theta2);
    y = y0 + r * sin(theta2);
    
    % 输出目标点的坐标
    target_point = [x, y];
end
