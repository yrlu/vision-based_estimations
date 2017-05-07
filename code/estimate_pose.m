% function [pos, q, Rc_w, Tc_w, Rw_c, Tw_c, pts_w, pts_c] = estimate_pose(sensor, varargin)
function [pos, q, Rw_b] = estimate_pose(sensor, varargin)
%ESTIMATE_POSE 6DOF pose estimator based on apriltags
%   sensor - struct stored in provided dataset, fields include
%          - is_ready: logical, indicates whether sensor data is valid
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
%              estimate_pose_handle = ...
%                  @(sensor) estimate_pose(sensor, your personal input arguments);
%   pos - 3x1 position of the quadrotor in world frame
%   q   - 4x1 quaternion of the quadrotor [w, x, y, z] where q = w + x*i + y*j + z*k

% estimate camera pose (R,T) pair such that,
%   (X_c;Y_c;Z_c)  = (R, T) * (X_w;Y_w;Z_w)

if(numel(sensor.id)==0)
    pos=[];
    q=[];
    return
end
K = [311.0520 0 201.8724; 0 311.3885 113.6210; 0 0 1];
inv_K = inv(K);
% T^b_c translation from frame c to frame b (w.r.t. frame b)
Tb_c = [0.02*sqrt(2); -0.02*sqrt(2); -0.03];
% R^b_c rotation from frame c to frame b (w.r.t. frame b)
Rb_c = [sqrt(2)/2, -sqrt(2)/2, 0; -sqrt(2)/2, -sqrt(2)/2, 0; 0, 0, -1];
%%%
pts_w = [get_tag_coords2(sensor.id); zeros(1, numel(sensor.id)*5)]; 
pts_c = [sensor.p0, sensor.p1, sensor.p2, sensor.p3, sensor.p4];
% pts_w = [get_tag_coords2(sensor.id); zeros(1, numel(sensor.id)*3)]; 
% pts_c = [sensor.p0, sensor.p1, sensor.p2];
[Rc_w, Tc_w] = estimate_cam_RT(pts_c, pts_w, inv_K);
[Rw_b, Tw_b] = get_body_pose(Rc_w, Tc_w, Rb_c, Tb_c);
pos = Tw_b;
q = rot2quat(Rw_b);
end
