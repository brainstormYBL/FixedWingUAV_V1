function [res] = calculate_point_line_cir(center_cir,radius_cir,k_line,bias_line)
x0 = center_cir(1);  
y0 = center_cir(2); 
r = radius_cir;   
m = k_line;  
b = bias_line;   
A = 1 + m^2;
B = 2 * m * b - 2 * m * y0 - 2 * x0;
C = x0^2 + y0^2 + b^2 - 2 * b * y0 - r^2;
delta = B^2 - 4 * A * C;  
if delta < 0
    x = (-B) / (2 * A);
    y = m * x + b;
    res = [x,y];
elseif delta == 0
    x = -B / (2 * A);
    y = m * x + b;
    res = [x,y];
else
    x1 = (-B + sqrt(delta)) / (2 * A);
    y1 = m * x1 + b;
    res = [x1,y1];
end
end