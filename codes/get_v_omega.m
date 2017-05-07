function [v_omega] = get_v_omega(Xc, Yc, Zc, oldXc, oldYc, oldZc, Rc_w, Tc_w, oldRc_w, oldTc_w, dt)
n = numel(Xc);
XYZw = [Rc_w', -Rc_w'*Tc_w]*[Xc; Yc; Zc; ones(1, n)];
oldXYZw = [Rc_w', -Rc_w'*Tc_w]*[oldXc; oldYc; oldZc; ones(1, n)];
% X = XYZw(1,:);
% Y = XYZw(2,:);
% Z = XYZw(3,:);
% oldX = oldXYZw(1,:);
% oldY = oldXYZw(2,:);
% oldZ = oldXYZw(3,:);

% T^b_c translation from frame c to frame b (w.r.t. frame b)
Tb_c = [0.02*sqrt(2); -0.02*sqrt(2); -0.03];
% R^b_c rotation from frame c to frame b (w.r.t. frame b)
Rb_c = [sqrt(2)/2, -sqrt(2)/2, 0; -sqrt(2)/2, -sqrt(2)/2, 0; 0, 0, -1];

X = Xc;
Y = Yc;
Z = Zc;
oldX = oldXc;
oldY = oldYc;
oldZ = oldZc;

x = X./Z;
y = Y./Z;

dot_X = (X-oldX)/dt;
dot_Y = (Y-oldY)/dt;
dot_Z = (Z-oldZ)/dt;

% \dot{X_c}/Z_c
dot_X_Z = dot_X./Z;
% \dot{Y_c}/Z_c
dot_Y_Z = dot_Y./Z;
% \dot{Z_c}/Z_c
dot_Z_Z = dot_Z./Z;

dot_x = dot_X_Z - dot_Z_Z.*x;
dot_y = dot_Y_Z - dot_Z_Z.*y;

[A] = get_A(x',y',Z');
v_omega_c = A\[dot_x';dot_y'];
% convert to world frame
Rw_c = Rc_w';
Tw_c = -Rc_w'*Tc_w;
oldRw_c = oldRc_w';
oldTw_c = -oldRc_w'*Tw_c;
dot_Tw_c = (Tw_c - oldTw_c)/dt;

v_omega = [Rw_c, -Rw_c*skew(Tw_c); zeros(3,3), Rw_c]*v_omega_c;

v_c = v_omega(1:3);
omega_c = v_omega(4:end);

v_b = Rb_c*v_c;
Rb_w = Rb_c*Rc_w;
v_w = Rb_w'*v_b;
v_omega(1:3) = v_w;
Rw_c*mean([dot_X; dot_Y; dot_Z]')';
% v_w = Rw_c*v_c;
% omega_w = Rw_c*omega_c;
% v_omega = [v_w; omega_w];

% omega_c = v_omega(4:6);
% omega_w = inv_skew(Rc_w*skew(omega_c)*Rc_w');
% v_w = cross(omega_w, Tw_c) + Rw_c*v_c + dot_Tw_c;
% v_w = skew(omega_w)*Rw_c*Tc_w + Rw_c*v_c + dot_Tw_c;
% v_omega = [v_w; omega_w];
% v_w = [Rc_w', -Rc_w'*Tc_w]*[v_omega(1:3); 1];
% omega_w = [Rc_w', -Rc_w'*Tc_w]*[v_omega(4:6); 1];
% v_omega = [v_w; omega_w];
end