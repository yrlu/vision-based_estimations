function [R, T] = estimate_cam_RT(UV, XYZ, inv_K)
% estimate the rotation matrix R and translation matrix T, such that,
%   (X_c;Y_c;Z_c)  = (R, T) * (X_w;Y_w;Z_w)
%   (U,V,1) = K*(X_c;Y_c;Z_c)
% given n corresponding points, n>=4
% 
% @input        UV      2xn coordinates in picture
%               XYZ     3xn coordinates in the world frame
%               inv_K   inverse K matrix
% @output       H       3x4 H matrix
%
% Yiren Lu (luyiren@seas.upenn.edu), GRASP, UPenn, 2017

% display '---'
% tic
n = size(UV,2);
A = zeros(2*n, 9);
A(1:n, 1) = -XYZ(1, :);
A(1:n, 2) = -XYZ(2, :);
A(1:n, 3) = -1;
A(1:n, 7) = XYZ(1,:).*UV(1,:);
A(1:n, 8) = XYZ(2,:).*UV(1,:);
A(1:n, 9) = UV(1,:);

A(n+1:end, 4) = -XYZ(1, :);
A(n+1:end, 5) = -XYZ(2, :);
A(n+1:end, 6) = -1;
A(n+1:end, 7) = XYZ(1,:).*UV(2,:);
A(n+1:end, 8) = XYZ(2,:).*UV(2,:);
A(n+1:end, 9) = UV(2,:);
% toc
% tic
[U S V] = svd(A, 'econ');
% toc
h = V(:,end);
H = reshape(h,3,3)';

if H(3,3) < 0
    H = -H;
end
% tic
r12_T = inv_K*H;
% toc
h1 = r12_T(:,1);
h2 = r12_T(:,2);
h3 = r12_T(:,3);
USV = [h1, h2, cross(h1,h2)];
% tic
[U,S,V] = svd(USV);
% toc
% tic
R = U*[1,0,0;0,1,0;0,0,det(U*V')]*V';
T = h3/norm(h1);
% toc