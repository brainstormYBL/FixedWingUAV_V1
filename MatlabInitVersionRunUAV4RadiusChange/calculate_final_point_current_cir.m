function [point_final] = calculate_final_point_current_cir(center_uav_current,center_uav_next,radius_current,radius_next)
x_ri = center_uav_current(1);
y_ri = center_uav_current(2);
x_rf = center_uav_next(1);
y_rf = center_uav_next(2);
r_i = radius_current;
r_f = radius_next;
if r_i == r_f
    if y_rf == y_ri
        point_final = [x_ri, y_ri + r_i];
    else
        theta = atan((x_rf - x_ri) / (y_rf - y_ri));
        syms x y;
        f1 = tan(pi / 2 - theta) * (y - y_ri) / (x - x_ri) + 1;
        f2 = (x - x_ri) .^ 2 + (y - y_ri) .^ 2 - r_i .^ 2;
        [x,y] = solve(f1,f2); 
        x = double(x);
        y = double(y);
        [~,n] = min(y);
        point_final = [x(n), y(n)];
    end
else
    C = x_ri - (x_ri * r_f - x_rf * r_i) / (r_f - r_i);
    D = -y_ri - (y_rf * r_i - y_ri * r_f) / (r_f - r_i);
    temp = (C ^ 2 * D ^ 2 - (r_i ^ 2 - C ^ 2) * (r_i ^ 2 - D ^ 2));
    if r_i > r_f
        A = (-C * D + sqrt(temp)) / (C ^ 2 - r_i ^ 2);
    else
        A = (-C * D - sqrt(temp)) / (C ^ 2 - r_i ^ 2);
    end
    B = ((-y_rf + A * x_rf) * r_i - (-y_ri + A * x_ri) * r_f) / (r_f - r_i);
    x_f = (x_ri - (B - y_ri) * A) / (A ^ 2 + 1);
    theta = acos((x_ri - x_f) / r_i);
    y_f = y_ri - r_i * sin(theta);
    point_final = [x_f, y_f];
end
end