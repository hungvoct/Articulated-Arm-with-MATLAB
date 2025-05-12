function [theta] = GetInverse(app)
    % Giới hạn góc (rad)
    theta1_limits = deg2rad([-180, 180]);
    theta2_limits = deg2rad([0, 180]);
    theta3_limits = deg2rad([-90, 90]);

    % Thông số articulated arm
    d1 = 0.5; 
    a2 = 0.5; 
    a3 = 0.5; 
    
    % Tọa độ end-effector
    pwx = app.xset.Value;
    pwy = app.yset.Value;
    pwz = app.zset.Value - d1;

    % Tính toán theta3
    c_theta3 = (pwx^2 + pwy^2 + pwz^2 - a2^2 - a3^2) / (2 * a2 * a3);
    % Kiểm tra cos(theta3) > 1?
    if abs(c_theta3) > 1
        uialert(app.UIFigure, 'Out of Workspace', 'Warning');
        theta = []; % Trả giá trị theta rỗng
        return;
    end

    % Hai nghiệm của theta3
    s_theta3 = sqrt(max(0, 1 - c_theta3^2)); %
    theta3_1 = atan2(s_theta3, c_theta3);
    theta3_2 = atan2(-s_theta3, c_theta3);
    
    theta_candidates = []; % Lưu tất cả nghiệm khả thi

    for theta3 = [theta3_1, theta3_2]
        % Tính toán theta2
        k1 = a2 + a3 * cos(theta3);
        k2 = a3 * sin(theta3);
        r = sqrt(pwx^2 + pwy^2);
        c_theta2 = (r * k1 + pwz * k2) / (k1^2 + k2^2);
        s_theta2 = (pwz * k1 - r * k2) / (k1^2 + k2^2);

        % Kiểm tra giới hạn tính toán
        if abs(c_theta2) > 1 || abs(s_theta2) > 1
            continue;
        end

        theta2_1 = atan2(s_theta2, c_theta2);
        theta2_2 = atan2(-s_theta2, c_theta2);

        % Tính theta1
        for theta2 = [theta2_1, theta2_2]
            theta1 = atan2(pwy, pwx);
            
            % Kiểm tra giới hạn góc
            if theta1 >= theta1_limits(1) && theta1 <= theta1_limits(2) && ...
               theta2 >= theta2_limits(1) && theta2 <= theta2_limits(2) && ...
               theta3 >= theta3_limits(1) && theta3 <= theta3_limits(2)
                theta_candidates = [theta_candidates; [theta1, theta2, theta3]]; 
            end
        end
    end

    % Nếu không có nghiệm hợp lệ
    if isempty(theta_candidates)
        uialert(app.UIFigure, 'Out of Workspace', 'Warning');
        theta = [];
        return;
    end

    % Chọn nghiệm có đường đi của end-effector nhỏ nhất
    theta1_current = app.theta1_current;
    theta2_current = app.theta2_current;
    theta3_current = app.theta3_current;      
    x_target = app.xset.Value;
    y_target = app.yset.Value;
    z_target = app.zset.Value;
    target_position = [x_target, y_target, z_target];

    min_distance = inf;
    best_theta = [];

    for i = 1:size(theta_candidates, 1)
        theta_candidate = theta_candidates(i, :);
        position_candidate = GetForward(theta_candidate(1), theta_candidate(2), theta_candidate(3));
        % Check lại động học thuận
        if norm(position_candidate - target_position) < 1e-4
            distance = abs(theta_candidate(1) - theta1_current) + ...
                       abs(theta_candidate(2) - theta2_current) + ...
                       abs(theta_candidate(3) - theta3_current);

            if distance < min_distance
                min_distance = distance;
                best_theta = theta_candidate;
            end
        end
    end
    % Nếu không có nghiệm phù hợp
    if isempty(best_theta)
        uialert(app.UIFigure, 'Không tìm thấy nghiệm theta phù hợp', 'Error');
        theta = [];
        return;
    end

    theta = best_theta;
end
