function [radius_uav] = calculate_radius_uavs(radius_mgus,height_uav, theta)
radius_mgus = reshape(radius_mgus, [], 1);
height_uav = reshape(height_uav, [], 1);
radius_uav = radius_mgus - height_uav .* theta;
for index_uav = 1:length(height_uav(:))
    if radius_uav(index_uav) < 200
        radius_uav(index_uav) = 200;
    end
end
end