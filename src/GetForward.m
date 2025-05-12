function [forward] = GetForward(theta1, theta2, theta3)
    % Giới hạn góc (rad)
    theta1_limits = deg2rad([-180, 180]);
    theta2_limits = deg2rad([0, 180]);
    theta3_limits = deg2rad([-90, 90]);
    
    % Kiểm tra giới hạn góc
    if theta1 < theta1_limits(1) || theta1 > theta1_limits(2) || ...
       theta2 < theta2_limits(1) || theta2 > theta2_limits(2) || ...
       theta3 < theta3_limits(1) || theta3 > theta3_limits(2)
        uialert(app.UIFigure, 'Out of Workspace', 'Warning');
        forward = []; % Trả giá trị rỗng khi vượt giới hạn
        return;
    end

    % Các thông số động học (đơn vị: mét, radian)
    d1 = 0.5; a1 = 0; alpha1 = pi/2;
    d2 = 0; a2 = 0.5; alpha2 = 0;
    d3 = 0; a3 = 0.5; alpha3 = 0;

    % Tọa độ base
    x0 = 0; y0 = 0; z0 = 0;
    base_pose = [x0; y0; z0; 1];

    % Ma trận DH
    A1 = DH_Matrix(d1, theta1, a1, alpha1);
    A2 = DH_Matrix(d2, theta2, a2, alpha2);
    A3 = DH_Matrix(d3, theta3, a3, alpha3);

    % Ma trận T
    T = A1 * A2 * A3;
    
    % Tọa độ end-effector
    end_pose = T * base_pose;
    x3 = end_pose(1);
    y3 = end_pose(2);
    z3 = end_pose(3);

    forward = [x3, y3, z3];
end
