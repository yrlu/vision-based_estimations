function [Xc, Yc, Zc, XYZw] = get_XYZc(Rc_w, Tc_w, points, invK)
% estimate points w.r.t. the camera frame and the world frame

u = points(:,1);
v = points(:,2);
n = numel(u);

% [Xc/lambda, Yc/lambda, Zc/lambda];
XYZc_lambda = invK*[u';v';ones(1,n)];
% compute lamda
tmp1 = Rc_w'*Tc_w;
tmp1 = tmp1(3);
tmp2 = Rc_w'*XYZc_lambda;
tmp2 = tmp2(3,:);
lambdas = tmp1./tmp2;

Xc = XYZc_lambda(1,:).*lambdas;
Yc = XYZc_lambda(2,:).*lambdas;
Zc = XYZc_lambda(3,:).*lambdas;

XYZw = [Rc_w', -Rc_w'*Tc_w]*[Xc; Yc; Zc; ones(1, n)];
end