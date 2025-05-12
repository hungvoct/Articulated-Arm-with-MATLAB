function workspace_points = ComputeWorkspace(theta1_limits, theta2_limits, theta3_limits, resolution)
    % Tạo các giá trị theta theo độ phân giải
    theta1_values = linspace(theta1_limits(1), theta1_limits(2), resolution);
    theta2_values = linspace(theta2_limits(1), theta2_limits(2), resolution);
    theta3_values = linspace(theta3_limits(1), theta3_limits(2), resolution);

    % Tính tổng số điểm trong không gian làm việc
    total_points = resolution^3;
    workspace_points = zeros(total_points, 3); % Ma trận để lưu trữ các điểm [x, y, z]

    % Biến chỉ số cho ma trận kết quả
    point_index = 1;

    % Duyệt qua tất cả các giá trị của theta1, theta2, theta3
    for i = 1:resolution
        for j = 1:resolution
            for k = 1:resolution
                % Lấy giá trị theta tương ứng
                theta1 = theta1_values(i);
                theta2 = theta2_values(j);
                theta3 = theta3_values(k);

                % Tính động học thuận
                point = GetForward(theta1, theta2, theta3); % Giả sử trả về [x, y, z]

                % Lưu kết quả vào ma trận workspace_points
                workspace_points(point_index, :) = point;

                % Tăng chỉ số
                point_index = point_index + 1;
            end
        end
    end
end
