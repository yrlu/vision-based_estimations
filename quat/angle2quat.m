function q = angle2quat(z, y, x)
%ANGLE2QUAT Convert Euler angles to a quaternion.
%   Q = ANGLE2QUAT(Z, Y, X) converts Euler angles Z, Y, X, into an
%   equivalent quaternion Q.

thetas = [z(:) y(:) x(:)];

c = cos(thetas/2);
s = sin(thetas/2);

q = [c(:,1).*c(:,2).*c(:,3) + s(:,1).*s(:,2).*s(:,3), ...
     c(:,1).*c(:,2).*s(:,3) - s(:,1).*s(:,2).*c(:,3), ...
     c(:,1).*s(:,2).*c(:,3) + s(:,1).*c(:,2).*s(:,3), ...
     s(:,1).*c(:,2).*c(:,3) - c(:,1).*s(:,2).*s(:,3)];