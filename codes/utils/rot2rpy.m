function [roll, pitch, yaw] = rot2rpy(rot)
% By Yiren Lu at Univ. of Pennsylvania, Feb 8, 2016
% Reference: http://planning.cs.uiuc.edu/node103.html

yaw = atan2(rot(2,1,:),rot(1,1,:));
pitch = atan2(-rot(3,1,:),sqrt(rot(3,2,:).^2+rot(3,3,:).^2));
roll = atan2(rot(3,2,:),rot(3,3,:));

roll = reshape(roll, [size(roll,3),1]);
yaw = reshape(yaw, [size(yaw,3),1]);
pitch = reshape(pitch, [size(pitch,3),1]);
end