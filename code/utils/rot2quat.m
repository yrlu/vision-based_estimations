function [q] = rot2quat(R)
% Rotation matrix R to quaternion representation
% @input    R   rotation matrix \in SO(3) (det(R) == 1)
% @output   q   1x4 converted quaternion

% Yiren Lu, GRASP, UPenn, 2017

qw = sqrt(1 + trace(R))/2;
qx = (R(3,2)-R(2,3))/(4*qw);
qy = (R(1,3)-R(3,1))/(4*qw);
qz = (R(2,1)-R(1,2))/(4*qw);
q = [qw, qx, qy, qz];