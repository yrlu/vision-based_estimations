function [vel, omg] = estimate_vel(sensor, varargin)
%ESTIMATE_VEL 6DOF velocity estimator
%   sensor - struct stored in provided dataset, fields include
%          - is_ready: logical, indicates whether sensor data is valid
%          - t: timestamp
%          - rpy, omg, acc: imu readings, you should not use these in this phase
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
%              estimate_vel_handle = ...
%                  @(sensor) estimate_vel(sensor, your personal input arguments);
%   vel - 3x1 velocity of the quadrotor in world frame
%   omg - 3x1 angular velocity of the quadrotor

if isempty(numel(sensor.id)) | ~sensor.is_ready | isempty(sensor.img)
    vel = [];
    omg = [];
    return
end

persistent pointTracker;
persistent oldXc;
persistent oldYc;
persistent oldZc;
persistent oldSensor;
persistent invK;
persistent cam_invK;
persistent framei;
persistent olddt;
persistent oldv;
persistent oldomg;

N_tracking = 150; % SURF
% N_tracking = 200; % FAST

if isempty(oldSensor) | oldSensor.t > sensor.t
    disp 'init'
    % initialize last frame
    framei = 1;
%     olddt = sensor.t;
    olddt = 0.02;
    oldv = zeros(3,1);
    oldomg = zeros(3,1);
    % init KLT
    pointTracker = vision.PointTracker('MaxBidirectionalError', 2);
    corners = detectFASTFeatures(sensor.img);
    points = corners.selectStrongest(N_tracking);
    points = double(points.Location);
    initialize(pointTracker, points, sensor.img);
    
    % init constants
%     K = [311.0520 0 201.8724; 0 311.3885 113.6210; 0 0 1];
    K = [314.1779 0         199.4848; 0         314.2218  113.7838; 0         0         1];
    invK = inv(K);
    
    cam_K = [311.0520 0 201.8724; 0 311.3885 113.6210; 0 0 1];
    cam_invK = inv(cam_K);
    
    % estimate poses
%     [Tw_b, q, Rc_w, Tc_w, Rw_c, Tw_c, pts_w, pts_c] = estimate_pose(sensor);
    [Rc_w, Tc_w] = estimate_cam_pose(sensor, cam_invK);
    [Xc, Yc, Zc, ~] = get_XYZc(Rc_w, Tc_w, points, invK);
    
    % backup
    oldXc = Xc;
    oldYc = Yc;
    oldZc = Zc;
    oldSensor = sensor;
    
    vel = zeros(3,1);
    omg = zeros(3,1);
    return
end

% disp '----'
% tic
points = step(pointTracker, sensor.img);
% toc
[h,w]=size(sensor.img);
ids = points(:,1) > w*0.12 & points(:,1) < w*0.88 & points(:,2) > h*0.12 & points(:,2) < h*0.88;
points = points(ids,:);

% estimate pose
% tic
% [Tw_b, q, Rc_w, Tc_w, Rw_c, Tw_c, pts_w, pts_c] = estimate_pose(sensor);
[Rc_w, Tc_w] = estimate_cam_pose(sensor, cam_invK);
[Xc, Yc, Zc, ~] = get_XYZc(Rc_w, Tc_w, points, invK);
alpha = 0.05;
% dt = sensor.t - oldSensor.t;
dt = alpha*(sensor.t - oldSensor.t) + (1-alpha)*olddt;
% framei
% dt = 0.02;
% toc
% tic
x = Xc./Zc;
y = Yc./Zc;

% oldx = oldXc./oldZc;
% oldy = oldYc./oldZc;

oldx = oldXc(ids)./oldZc(ids);
oldy = oldYc(ids)./oldZc(ids);

% solve for the velocities
dot_x = [x - oldx]'./dt;
dot_y = [y - oldy]'./dt;
A_all = get_A(x',y',Zc');
% v_omega_c = A_all\[dot_x;dot_y];
v_omega_c = ransac(A_all, x, y, Zc, dot_x, dot_y);
% toc
% tic
% redetect the points
% if mod(framei, 2) == 0 | size(points,1) < 100
if mod(framei, 2) == 0
% if size(points,1) < 140
corners = detectFASTFeatures(sensor.img);
points = corners.selectStrongest(N_tracking);
points = double(points.Location);

[h,w]=size(sensor.img);
ids = points(:,1) > w*0.12 & points(:,1) < w*0.88 & points(:,2) > h*0.12 & points(:,2) < h*0.88;
% ids = points(:,1) > w*0.15 & points(:,1) < w*0.85 & points(:,2) > h*0.15 & points(:,2) < h*0.85;
points = points(ids,:);

% [Tw_b, q, Rc_w, Tc_w, Rw_c, Tw_c, pts_w, pts_c] = estimate_pose(sensor);
[Rc_w, Tc_w] = estimate_cam_pose(sensor, cam_invK);
[Xc, Yc, Zc, ~] = get_XYZc(Rc_w, Tc_w, points, invK);
end
setPoints(pointTracker, points);
% toc
% display
% if mod(framei, 1) == 0
% plot_data3(sensor, Rw_b, Tw_b, points, XYZw);
% end

% back up current frame
framei = framei + 1;
oldXc = Xc;
oldYc = Yc;
oldZc = Zc;
oldSensor = sensor;
olddt = dt;

% converting the velocities from the camera frame to the world frame
vel = Rc_w'*v_omega_c(1:3);
omg = Rc_w'*v_omega_c(4:6);

a=1/exp(norm(vel-oldv));
vel = vel*a + oldv*(1-a);
a=1/exp(norm(omg-oldomg)/100);
omg = omg*a + oldomg*(1-a);
oldv = vel;
oldomg = omg;
end
