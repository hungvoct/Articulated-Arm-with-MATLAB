function updateRobotPose(app, theta1, theta2, theta3)
    % Giới hạn góc (rad)
    theta1_limits = deg2rad([-180, 180]);
    theta2_limits = deg2rad([0, 180]);
    theta3_limits = deg2rad([-90, 90]);
    
    % Kiểm tra giới hạn góc
    if theta1 < theta1_limits(1) || theta1 > theta1_limits(2) || ...
       theta2 < theta2_limits(1) || theta2 > theta2_limits(2) || ...
       theta3 < theta3_limits(1) || theta3 > theta3_limits(2)
        uialert(app.UIFigure, 'Out of Workspace', 'Warning');
    else
        d1 = 0.5; a1 = 0; alpha1 = pi/2;
        d2 = 0; a2 = 0.5; alpha2 = 0;
        d3 = 0; a3 = 0.5; alpha3 = 0;
    
        x0 = 0;
        y0 = 0;
        z0 = 0;
    
        base_pose = [x0; y0; z0; 1];
    
        % Ma trận chuyển đổi cho từng khớp
        A1 = DH_Matrix(d1, theta1, a1, alpha1);
        A2 = DH_Matrix(d2, theta2, a2, alpha2);
        A3 = DH_Matrix(d3, theta3, a3, alpha3);
    
        % Ma trận DH từ base tới end-effector
        T = A1 * A2 * A3;
        
        l1_pose = A1*base_pose;
        x1 = l1_pose(1);
        y1 = l1_pose(2);
        z1 = l1_pose(3);
    
        l2_pose = A1*A2*base_pose;
        x2 = l2_pose(1);
        y2 = l2_pose(2);
        z2 = l2_pose(3);
        
        % Tọa độ của end-effector
        end_pose = T*base_pose;
        x3 = end_pose(1);
        y3 = end_pose(2);
        z3 = end_pose(3);
    
        %Tính Roll - Pitch - Yaw
        rpy = CaculateR_P_Y(T);
        roll = rpy(1);
        pitch = rpy(2);
        yaw = rpy(3);

        % Xóa trục cũ và thiết lập chế độ 3D
        cla(app.RobotGraph);  % Xóa trục trước khi vẽ mới
        hold(app.RobotGraph, 'on');
        grid(app.RobotGraph, 'on'); % Bật lưới
        view(app.RobotGraph, 3);    % Đặt chế độ xem 3D
    
       % Vẽ robot trong không gian 3D
        plot3(app.RobotGraph, [0, x0], [0, y0], [0, z0], 'black', 'LineWidth', 4); % Đoạn nối 0
        plot3(app.RobotGraph, [x0, x1], [y0, y1], [z0, z1], 'r-', 'LineWidth', 4); % Đoạn nối 1
        plot3(app.RobotGraph, [x1, x2], [y1, y2], [z1, z2], 'g-', 'LineWidth', 4); % Đoạn nối 2
        plot3(app.RobotGraph, [x2, x3], [y2, y3], [z2, z3], 'b-', 'LineWidth', 4); % Đoạn nối 3
        
        %Vẽ base
        % Thiết lập bán kính và số lượng điểm cho hình tròn
        r = 0.3; % Bán kính hình tròn
        goc = linspace(0, 2*pi, 100); % Góc từ 0 đến 2*pi
        x_circle = x0 + r * cos(goc); % Tọa độ x của hình tròn
        y_circle = y0 + r * sin(goc); % Tọa độ y của hình tròn
        plot(app.RobotGraph, x_circle, y_circle, 'b-', 'LineWidth', 2); % Base

        plot3(app.RobotGraph, x1, y1, z1, 'ko', 'MarkerSize', 4, 'MarkerFaceColor', 'k'); % Joint 1
        plot3(app.RobotGraph, x2, y2, z2, 'ko', 'MarkerSize', 4, 'MarkerFaceColor', 'k'); % Joint 2
        plot3(app.RobotGraph, x3, y3, z3, 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k'); % End-effector
        axis(app.RobotGraph, [-2 2 -2 2 0 2]);
        hold(app.RobotGraph, 'off'); % Kết thúc giữ đồ thị
        
        app.xValue.Value = x3;
        app.yValue.Value = y3;
        app.zValue.Value = z3;
        %Value_2 là ô chứa giá trị nằm ở khung Differential Kinematics
        app.xValue_2.Value = x3;
        app.yValue_2.Value = y3;
        app.zValue_2.Value = z3;
        app.rollValue.Value = roll;
        app.pitchValue.Value = pitch;
        app.yawValue.Value = yaw;
    end
end
