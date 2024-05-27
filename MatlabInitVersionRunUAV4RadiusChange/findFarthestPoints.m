function [index] = findFarthestPoints(coordinates, num_point)
dis = zeros(num_point, num_point);
for index_1 = 1:num_point
    for index_2 = 1:num_point
        if index_1 == index_2
            dis(index_1, index_2) = 0;
        else
            dis(index_1, index_2) = sqrt(sum((coordinates(index_1, :) - coordinates(index_2, :)) .^ 2));
        end
    end
end
max_value = 0;
index_row = 1;
index_col = 1;
for index_1 = 1:num_point
    for index_2 = 1:num_point
        if dis(index_1, index_2) > max_value
            max_value = dis(index_1, index_2);
            index_row = index_1;
            index_col = index_2;
        end
    end
end
index = [index_row, index_col];
end
