function [X, Z, rpy] = ekf1(sensor, vic, varargin)
% EKF1 Extended Kalman Filter with Vicon velocity as inputs
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
%   vic    - struct for storing vicon linear velocity in world frame and
%            angular velocity in body frame, fields include
%          - t: vicon timestamp
%          - vel = [vx; vy; vz; wx; wy; wz]
%   varargin - any variables you wish to pass into the function, could be
%              a data structure to represent the map or camera parameters,
%              your decision. But for the purpose of testing, since we don't
%              know what inputs you will use, you have to specify them in
%              init_script by doing
%              ekf1_handle = ...
%                  @(sensor, vic) ekf1(sensor, vic, your input arguments);
%
% OUTPUTS:
% X - nx1 state of the quadrotor, n should be greater or equal to 6
%     the state should be in the following order
%     [x; y; z; qw; qx; qy; qz; other states you use]
%     we will only take the first 7 rows of X
% OPTIONAL OUTPUTS:
% Z - mx1 measurement of your pose estimator, m shoulb be greater or equal to 7
%     the measurement should be in the following order
%     [x; y; z; qw; qx; qy; qz; other measurement you use]
%     note that this output is optional, it's here in case you want to log your
%     measurement


persistent mu;
persistent muv;
persistent sigma;
persistent A;
persistent B;
persistent C;
persistent Q;
persistent R;
persistent oldt;
persistent oldZ;
persistent oldvic;


if isempty(A) | oldt > sensor.t
    A = eye(6); 
    B = eye(6);
    C = eye(6);
%     Q = diag([[1, 1, 1]*1e-16, [0.1, 1, 100]*1e-10]); % prediction noise
    Q = diag([[1, 1, 1]*1e-16, [1, 1, 1]*1e-11]); % prediction noise
    R = diag([[1, 1, 1]*1e-8, [1, 1, 1]*1e-9]); % measurement noise
    oldt = sensor.t;

    X = zeros(7,1);
    Z = zeros(7,1);
    rpy = zeros(3,1);
    
    
    oldZ = Z;
    
    oldvic = {};
    oldvic.vel = zeros(6,1);
    oldvic.t = sensor.t;
    mu = zeros(6,1);
    muv = zeros(6,1);
    sigma = eye(6)*1e-3;
    return;
end

if isempty(sensor.id)
    % only prediction, no update
    dt = vic.t - oldvic.t;
    u = vic.vel;  
    muv = A*muv+B*dt*u;
    rpy = muv(4:6);
    q_hat = angle2quat(rpy(3), rpy(2), rpy(1));
    X = [muv(1:3); q_hat'];
    Z = oldZ;
    oldvic = vic;
    return;
end

% both prediction and update
[pos, q, Rw_b] = estimate_pose(sensor);
[roll, pitch, yaw] = rot2rpy(Rw_b);

dt = sensor.t - oldt;
u = vic.vel;
% u = oldvic.vel;
mu_bar = A*mu+B*dt*u;
sigma_bar = A*sigma*A' + Q;
K = sigma_bar*C'*(C*sigma_bar*C' + R)^(-1);
mu = mu_bar + K*([pos;[roll, pitch, yaw]'] - C*mu_bar);
sigma = sigma_bar - K*C*sigma_bar;

Z = zeros(7,1);
rpy = mu(4:6);
% rpy = [roll, pitch, yaw]';
q_hat = angle2quat(rpy(3), rpy(2), rpy(1));
% q_hat = rot2quat(rpy2rot(rpy(1), rpy(2), rpy(3)));
X = [mu(1:3); q_hat'];

oldt = sensor.t;
oldZ = Z;

muv = mu;
oldvic = vic;
end
