function [uav_set] = update_uav_set(time_all, slot_current,num_uav)
uav_set = 1:1:num_uav;
for index_uav = 1:num_uav
    if slot_current > time_all(index_uav)
        uav_set = uav_set(uav_set ~= index_uav);
    end
end
end