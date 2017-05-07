function [X, Z, rpy] = ekf2(sensor, varargin)
% EKF2 Extended Kalman Filter with IMU as inputs
%
% INPUTS:
%   sensor - struct stored in provided dataset, fields include
%          - is_ready: logical, indicates whether sensor data is valid
%          - t: sensor timestamp
%          - rpy, omg, acc: imu readings
%          - img: uint8, 240x376 grayscale image
%          - id: 1xn ids of detected tags
%          - p0, p1, p2, p3, p4: 2xn pixel position of center and
%                                four corners of detected tags
%            Y
%            ^ P3 == P2
%            | || P0 ||
%            | P4 == P1
%            o---------> X
%   varargin - any variables you wish to pass into the function, could be
%              a data structure to represent the map or camera parameters,
%              your decision. But for the purpose of testing, since we don't
%              know what inputs you will use, you have to specify them in
%              init_script by doing
%              ekf1_handle = ...
%                  @(sensor) ekf2(sensor, your input arguments);
%
% OUTPUTS:
% X - nx1 state of the quadrotor, n should be greater or equal to 9
%     the state should be in the following order
%     [x; y; z; vx; vy; vz; qw; qx; qy; qz; other states you use]
%     we will only take the first 10 rows of X
% OPTIONAL OUTPUTS:
% Z - mx1 measurement of your pose estimator, m shoulb be greater or equal to 6
%     the measurement should be in the following order
%     [x; y; z; qw; qx; qy; qz; other measurement you use]
%     note that this output is optional, it's here in case you want to log your
%     measurement

USE_UKF = 0;

persistent mu;
persistent sigma;
persistent A;
persistent B;
persistent C;
persistent oldt;
persistent oldZ;
persistent oldX;

Q = diag([[1, 1, 1]*1e-7, [1, 1, 1]*2e-11]); % prediction noise
R = diag([[1, 1, 1]*1e-8, [1, 1, 1]*1e-9]); % measurement noise
    
if isempty(A) | oldt > sensor.t
    A = eye(6); 
    B = eye(6);
    C = eye(6);
    oldt = sensor.t;

    X = zeros(10,1);
    Z = zeros(7,1);
    rpy = zeros(3,1);
    
    oldX = X;
    oldZ = Z;
    
    mu = zeros(6,1);
    sigma = eye(6)*1e-3;
    return;
end


if isempty(sensor.id)
    X = oldX;
    Z = oldZ;
    return;
end


% both prediction and update
[pos, q, Rw_b] = estimate_pose(sensor);
[roll, pitch, yaw] = rot2rpy(Rw_b);

dt = sensor.t - oldt;
[vel, omg] = estimate_vel(sensor);
u = [vel; omg];
mu_bar = A*mu+B*dt*u;
sigma_bar = A*sigma*A' + Q;
K = sigma_bar*C'*(C*sigma_bar*C' + R)^(-1);
mu = mu_bar + K*([pos;[roll, pitch, yaw]'] - C*mu_bar);
sigma = sigma_bar - K*C*sigma_bar;

Z = zeros(7,1);
rpy = mu(4:6);

if USE_UKF == 1
[s] = ukf(q, omg, sensor.t);
if(s(1) < 0) 
    s(1:4) = -s(1:4);
end
X = [mu(1:3);vel; s(1:4)];
[yaw, pitch, roll] = quat2angle(s(1:4)');
rpy = [roll, pitch, yaw];
return
end

q_hat = angle2quat(rpy(3), rpy(2), rpy(1));
X = [mu(1:3);vel; q_hat'];

% Use the raw estimation
% X = [mu(1:3);vel; q'];
% [yaw, pitch, roll] = quat2angle(q);
% rpy = [roll, pitch, yaw];


oldt = sensor.t;
oldX = X;
oldZ = Z;

end

