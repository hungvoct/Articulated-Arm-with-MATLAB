function differentialKinematics(app)
    % Lấy tọa độ target
    app.xset.Value = app.xset2.Value;
    app.yset.Value = app.yset2.Value;
    app.zset.Value = app.zset2.Value;

    % Góc khớp hiện tại
    th1_0 = app.theta1Slider.Value;
    th2_0 = app.theta2Slider.Value;
    th3_0 = app.theta3Slider.Value;
    theta_start = [th1_0, th2_0, th3_0];

    % Tính toán động học ngược
    theta_goal = GetInverse(app);

    % Thông số chuyển động
    %T = app.tmaxValue.Value;
    dt = 0.1;
    vmax = app.vmaxValue.Value;
    amax = app.amaxValue.Value;
    goc_lech = theta_goal - theta_start;
    goc_lech_max = max(goc_lech);
    T = goc_lech_max/vmax + vmax/amax;
    %fprintf("Thoi gian chuyen dong T = %f\n",T);

    % Khởi tạo quỹ đạo (rỗng ban đầu)
    time = 0:dt:T;
    num_steps = length(time);
    theta_traj = []; % Mảng rỗng chứa góc xoay của các khớp tại từng thời điểm
    theta_dot_traj = [];
    theta_ddot_traj = [];
    vep = []; % Vận tốc theo x,y,z của end-effector
    ve = [];  % Vận tốc end-effector

    % Lưu lại các giá trị theta1, theta2, theta3
    theta1 = [];
    theta2 = [];
    theta3 = [];
    theta1dot = [];
    theta2dot = [];
    theta3dot = [];
    theta1ddot = [];
    theta2ddot = [];
    theta3ddot = [];

    % Tính quỹ đạo cho từng khớp
    for i = 1:3
        delta_theta = theta_goal(i) - theta_start(i);
        Tb = vmax / amax;
        if T > 2 * Tb      %Thời gian T đủ để đồ thị vận tốc tạo thành hình thang
            v_const = delta_theta / (T - Tb);
            if abs(v_const) > vmax         %Thời gian tăng tốc Tb không đủ để khớp đạt vận tốc vmax
                error('Không thể đạt được vận tốc tối đa cho khớp %d.', i);
            end
            Tb = abs(v_const) / amax;
        else
            error('Không thể đạt được vận tốc tối đa cho khớp %d. Nhập lại vận tốc.', i);
            %Tb = sqrt(abs(delta_theta) / amax);
            %v_const = amax * Tb;
        end

        % Tạo quỹ đạo
        for t_idx = 1:num_steps
            t = time(t_idx);
            if t < Tb
                % a = amax
                theta_traj(i, t_idx) = theta_start(i) + 0.5 * sign(delta_theta) * amax * t^2;
                theta_dot_traj(i, t_idx) = sign(delta_theta) * amax * t;
                theta_ddot_traj(i, t_idx) = sign(delta_theta) * amax;
            elseif t < (T - Tb)
                % a = 0
                theta_traj(i, t_idx) = theta_start(i) + 0.5 * sign(delta_theta) * amax * Tb^2 + sign(delta_theta) * v_const * (t - Tb);
                theta_dot_traj(i, t_idx) = sign(delta_theta) * v_const;
                theta_ddot_traj(i, t_idx) = 0;
            else
                % a = -amax
                t_dec = t - (T - Tb);
                theta_traj(i, t_idx) = theta_start(i) + 0.5 * sign(delta_theta) * amax * Tb^2 + sign(delta_theta) * v_const * (T - 2*Tb) + sign(delta_theta) * v_const*t_dec - 0.5*sign(delta_theta) * amax*(t_dec)^2;
                theta_dot_traj(i, t_idx) = sign(delta_theta) * v_const - sign(delta_theta) * amax * t_dec;
                theta_ddot_traj(i, t_idx) = -sign(delta_theta) * amax;
            end
        end
    end

    % Tính vận tốc end-effector tại từng thời điểm
    for t_idx = 1:num_steps
        J = Jacobian_Matrix(theta_traj(1, t_idx), theta_traj(2, t_idx), theta_traj(3, t_idx));
        % Kiểm tra singularities
        if (abs(det(J)) < 1e-6) && (theta_traj(1, t_idx)~=0 && theta_traj(2, t_idx)~=0 && theta_traj(3, t_idx)~=0)
            fprintf("Singularities at theta: [%f, %f, %f]\n", theta_traj(1, t_idx), theta_traj(2, t_idx), theta_traj(3, t_idx));
        end
        joint_velocities = theta_dot_traj(:, t_idx);
        vep(:, end+1) = J * joint_velocities; % Mỗi lần lặp, thêm một cột vào mảng chứa vận tốc
        ve(end+1) = norm(vep(:, end));       % Tính vận tốc dài end-effector
        
        % Cập nhật robot pose & orientation
        updateRobotPose_traj(app, theta_traj(1, t_idx), theta_traj(2, t_idx), theta_traj(3, t_idx));
        pause(dt); % pause

        % Cập nhật giá trị trên giao diện
        app.theta1Slider.Value = rad2deg(theta_traj(1, t_idx));
        app.theta2Slider.Value = rad2deg(theta_traj(2, t_idx));
        app.theta3Slider.Value = rad2deg(theta_traj(3, t_idx));
        app.theta1Value.Value = rad2deg(theta_traj(1, t_idx));
        app.theta2Value.Value = rad2deg(theta_traj(2, t_idx));
        app.theta3Value.Value = rad2deg(theta_traj(3, t_idx));

        % Cập nhật giá trị theta1, theta2, theta3
        theta1(end+1) = theta_traj(1, t_idx);
        theta2(end+1) = theta_traj(2, t_idx);
        theta3(end+1) = theta_traj(3, t_idx);
        theta1dot(end+1) = theta_dot_traj(1, t_idx);
        theta2dot(end+1) = theta_dot_traj(2, t_idx);
        theta3dot(end+1) = theta_dot_traj(3, t_idx);
        theta1ddot(end+1) = theta_ddot_traj(1, t_idx);
        theta2ddot(end+1) = theta_ddot_traj(2, t_idx);
        theta3ddot(end+1) = theta_ddot_traj(3, t_idx);

        % Vẽ đồ thị
        plot(app.VelocityGraph, time(1:t_idx), ve, 'r', 'LineWidth', 1.5); % V_end-effector
        plot(app.Theta1Graph, time(1:t_idx), rad2deg(theta1), 'b', 'LineWidth', 1.5); % theta1
        plot(app.Theta2Graph, time(1:t_idx), rad2deg(theta2), 'b', 'LineWidth', 1.5); % theta2
        plot(app.Theta3Graph, time(1:t_idx), rad2deg(theta3), 'b', 'LineWidth', 1.5); % theta3
        plot(app.Theta1dotGraph, time(1:t_idx), theta1dot, 'b', 'LineWidth', 1.5); % theta1dot
        plot(app.Theta2dotGraph, time(1:t_idx), theta2dot, 'b', 'LineWidth', 1.5); % theta2dot
        plot(app.Theta3dotGraph, time(1:t_idx), theta3dot, 'b', 'LineWidth', 1.5); % theta3dot
        plot(app.Theta1ddotGraph, time(1:t_idx), theta1ddot, 'b', 'LineWidth', 1.5); % theta1ddot
        plot(app.Theta2ddotGraph, time(1:t_idx), theta2ddot, 'b', 'LineWidth', 1.5); % theta2ddot
        plot(app.Theta3ddotGraph, time(1:t_idx), theta3ddot, 'b', 'LineWidth', 1.5); % theta3ddot
    end
end
