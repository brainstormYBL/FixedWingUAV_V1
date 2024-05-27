function arc_length = calculateArcLength(radius, center, point1, point2, clockwise)
    % radius: 圆的半径
    % center: 圆心坐标，格式为 [x0, y0]
    % point1: 第一个点坐标，格式为 [x1, y1]
    % point2: 第二个点坐标，格式为 [x2, y2]
    % clockwise: 方向，true表示顺时针，false表示逆时针
    
    % 计算两个点相对于圆心的极角
    theta1 = atan2(point1(2)-center(2), point1(1)-center(1));
    theta2 = atan2(point2(2)-center(2), point2(1)-center(1));
    % 将极角转换为角度
    theta1_deg = rad2deg(theta1);
    theta2_deg = rad2deg(theta2);
    
    % 调整角度范围为0到360度
    theta1_deg = mod(theta1_deg, 360);
    theta2_deg = mod(theta2_deg, 360);
    
    % 根据方向调整起点和终点的顺序
    if clockwise
        if theta1_deg > theta2_deg
            theta2_deg = theta2_deg + 360;
        end
    else
        if theta2_deg > theta1_deg
            theta1_deg = theta1_deg + 360;
        end
    end
    
    % 计算弧长
    arc_length = abs(radius * deg2rad(theta2_deg - theta1_deg));
end
