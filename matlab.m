% Stewart Platform Inverse Kinematics Simulation

% 1. Define base and platform attachment points (in base frame)
% Example: Hexagonal arrangement (units: mm)
R_base = 200;    % Radius of base
R_plat = 100;    % Radius of platform

theta_base = (0:60:300) * pi/180;
theta_plat = (30:60:330) * pi/180; % Rotated by 30 deg for symmetry

B = [R_base*cos(theta_base); R_base*sin(theta_base); zeros(1,6)]';   % 6x3
P = [R_plat*cos(theta_plat); R_plat*sin(theta_plat); zeros(1,6)]';   % 6x3

% 2. Desired pose of the platform (position and orientation)
% Position (x, y, z)
T = [0; 0; 300]; % 300 mm above base center

% Orientation (roll, pitch, yaw in radians)
roll = 0.1;   % about x
pitch = 0.2;  % about y
yaw = 0.3;    % about z

% Rotation matrix (ZYX Euler angles)
Rx = [1 0 0; 0 cos(roll) -sin(roll); 0 sin(roll) cos(roll)];
Ry = [cos(pitch) 0 sin(pitch); 0 1 0; -sin(pitch) 0 cos(pitch)];
Rz = [cos(yaw) -sin(yaw) 0; sin(yaw) cos(yaw) 0; 0 0 1];
R = Rz * Ry * Rx;

% 3. Compute actuator lengths
L = zeros(6,1);
for i = 1:6
    % Transform platform point to base frame
    Pi = R * P(i,:)' + T;
    % Vector from base to platform point
    Li = Pi - B(i,:)';
    L(i) = norm(Li);
end

% Display results
disp('Actuator lengths (mm):');
disp(L);

% Optional: Visualize the platform
figure; hold on; axis equal; grid on;
xlabel('X'); ylabel('Y'); zlabel('Z');
plot3(B(:,1), B(:,2), B(:,3), 'ro', 'MarkerSize', 8, 'DisplayName', 'Base');
plot3(P(:,1)+T(1), P(:,2)+T(2), P(:,3)+T(3), 'bo', 'MarkerSize', 8, 'DisplayName', 'Platform');
for i = 1:6
    Pi = R * P(i,:)' + T;
    plot3([B(i,1) Pi(1)], [B(i,2) Pi(2)], [B(i,3) Pi(3)], 'k-', 'LineWidth', 2);
end
legend;
title('Stewart Platform Inverse Kinematics');
