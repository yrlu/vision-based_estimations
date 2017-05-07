function [Rw_b, Tw_b] = get_body_pose(Rc_w, Tc_w, Rb_c, Tb_c)
% get body pose (w.r.t. world frame w) from (R^c_w, T^c_w, R^b_c, T^b_c)
% 
% @input        Rc_w    R^c_w, rotation of camera frame w.r.t. world frame
%               Tc_w    T^c_w, translation of camera frame w.r.t. world
%                       frame
%               Rb_c    R^b_c, rotation of body frame w.r.t. camera frame
%               Tb_c    T^b_c, translation of body frame w.r.t. camera
%                       frame

% R^b_w = R^b_c * R^c_w 
Rb_w = Rb_c*Rc_w;
% T^b_w = R^b_c*T^c_w + T^b_c
Tb_w = Rb_c*Tc_w + Tb_c;

% R^w_b = (R^b_w)'
Rw_b = Rb_w';
% T^w_b = -(R^b_w)'*Tb_w
Tw_b = -Rb_w'*Tb_w;