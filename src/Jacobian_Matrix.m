function [J] = Jacobian_Matrix(theta1, theta2, theta3)
    %Ma tran Jacobian voi d1 = d1 giong voi ma tran Jacobian khi d1 = 0
    %d1 = 0.5;
    a2 = 0.5;
    a3 = 0.5;
    j11 = -sin(theta1)*(a2*cos(theta2)+a3*cos(theta2+theta3));
    j12 = -cos(theta1)*(a2*sin(theta2)+a3*sin(theta2+theta3));
    j13 = -a3*cos(theta1)*sin(theta2+theta3);
    j21 = cos(theta1)*(a2*cos(theta2)+a3*cos(theta2+theta3));
    j22 = -sin(theta1)*(a2*sin(theta2)+a3*sin(theta2+theta3));
    j23 = a3*sin(theta1)*sin(theta2+theta3);
    j31 = 0;
    j32 = a2*cos(theta2)+a3*cos(theta2+theta3);
    j33 = a3*cos(theta2+theta3);
    %j41 = 0;
    %j42 = sin(theta1);
    %j43 = sin(theta1);
    %j51 = 0;
    %j52 = -cos(theta1);
    %j53 = -cos(theta1);
    %j61 = 1;
    %j62 = 0;
    %j63 = 0;
    
    J = [[j11 j12 j13]; [j21 j22 j23]; [j31 j32 j33]];
end
