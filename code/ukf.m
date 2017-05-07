function [s] = ukf(q, omg, t)
% UKF Unscented Kalman Filter with IMU as inputs
%
% @inputs:
%   sensor - q:     current estimation of the orientation
%          - omg:   current estimation of the angular velocity (omega)
%          - t:     sensor timestamp
% @outputs:
%       s       7x1 vector containing 4x 1quaternion represeting the
%               orientation and the 3x1 angular velocity

% @author   Yiren Lu
% @email    luyirenmax@gmail.com

persistent olds;
persistent oldtt;
persistent P;

n = 6;
QQ = diag([[1, 1, 1]*1e-10, [1, 1, 1]*1e-6]); % process noise cov (6*6)
RR = diag([[1, 1, 1]*1e-10, [1, 1, 1]*1e-10]); % measurement noise cov (6*6)

if isempty(olds) | oldtt > t
    olds = [1,0,0,0,0,0,0]';
    oldtt = t;
    P = diag([[1, 1, 1]*1e-12, [1, 1, 1]*1e-12]); % estimate error cov (6*6)
end

dt = t - oldtt;

% compute dq from oldomega and dt
w = olds(5:7);
w_norm = sqrt(sum(w.^2));
angle = w_norm*dt;
axs = w/(w_norm+eps);
dq = [cos(angle/2) axs'.*sin(angle/2)];
dq(isnan(dq)) = 0;

% R = eye(3)*cos(angle) + axs*axs'*(1-cos(angle)) + skew_sym(axs)*sin(angle);
% [r,p,y] = rot2rpy(R);
% R = rpy2rot(r,p,y);
% dq1 = rot2quat(R);
% [r,p,y] = rot2rpy(R);
% [yaw, pitch, roll] = dcm2angle(R);
% dq1 = rot2quat(rpy2rot(w(1)*dt, w(2)*dt, w(3)*dt));

% compute sigma points
S = chol(P+QQ);          % S 6 * 6
S = sqrt(2*n)*S;        
W = [S, -S];            % W 6 * 12

% convert 6d W into 7d sigma points X: 12*7
% X(:, 1:4) = vec2quat(W(1:3,:));
X(:, 1:4) = angle2quat(W(3,:), W(2,:), W(1,:));
X(:, 5:7) = W(4:6,:)';
X(:, 1:4) = quatmultiply(olds(1:4)', X(:,1:4));

% Process: Transformations of the sigma points 
Y(:,1:4) = quatmultiply(X(:,1:4),dq);
Y(:,5:7) = bsxfun(@plus, X(:,5:7), w');

[q_hat]=avg_quaternion_markley(Y(:,1:4));
x_k(1:4) = q_hat;
x_k(5:7) = mean(Y(:,5:7), 1);

% Wprime(:,1:3) = quat2vec(quatmultiply(Y(:,1:4), quatconj(x_k(1:4))))';
[rs,ps,ys] = quat2angle(quatmultiply(Y(:,1:4), quatconj(x_k(1:4))), 'XYZ');
Wprime(:,1:3) = [rs,ps,ys];
Wprime(:,4:6) = bsxfun(@minus, Y(:, 5:7), x_k(5:7));
P_k = Wprime'*Wprime/2/n;

% Measurement: Got from the input
% [~, q, ~] = estimate_pose(sensor);
% [~, omg] = estimate_vel(sensor);

% Z = [quat2vec(Y(:, 1:4))', Y(:, 5:7)];
[rs,ps,ys] = quat2angle(Y(:, 1:4), 'XYZ');
Z = [[rs,ps,ys], Y(:, 5:7)];
z_ = mean(Z);
[rs,ps,ys] = quat2angle(q, 'XYZ');
z = [[rs,ps,ys], omg'];
% z = [quat2vec(q)', omg'];

% Measurement Estimate Cov
Wz = bsxfun(@minus, Z, z_);
Pzz = Wz'*Wz/2/n;
Pvv = Pzz + RR;

% Cross correlation matrix
Z_sig = bsxfun(@minus, Z, z_);
Pxz = 1/2/n * Wprime'*Z_sig;

% Kalman Gain and Update
K = Pxz/Pvv;
P = P_k - K*Pvv*K';
v = z - z_;
Kv = K*v';

% s = z_' + K*(z - z_)';
s = zeros(7,1);
% s(1:4) = quatmultiply(x_k(1:4), vec2quat(Kv(1:3)));
s(1:4) = quatmultiply(x_k(1:4), angle2quat(Kv(3), Kv(2), Kv(1)));
s(5:7) = x_k(5:7) + Kv(4:6)';
% [z_', z']
olds = s;
oldtt = t;
end

function q = vec2quat(vec)
% vec   n*3
% q     n*4
    angle = sqrt(sum(vec(1:3,:).^2,1));
    ev = bsxfun(@rdivide, vec(1:3,:), angle);
    q = [cos(angle/2); bsxfun(@times, ev, sin(angle/2))]';
    q(isnan(q)) = 0;
    q(isinf(q)) = 0;
end

function vec = quat2vec(q)
% vec   n*3
% q     n*4
    q = quatnormalize(q)';
    angles = acos(q(1,:))*2; % 1*n;
    sins = sin(angles); % 1*n
    vec = bsxfun(@times, (bsxfun(@rdivide, q(2:4,:), sins))', sins')';
    vec(isnan(vec)) = 0;
    vec(isinf(vec)) = 0;
end
