function plot_data(data, cam_T, cam_R, pts, vicon_v)
% @input    data        struct of a data frame
%           cam_T     3x1 vector
%           cam_R       3x3 rotation matrix
%           pts         3xn keypoints on the april tags
%           vicon_v     1x12 vicon data
subplot(1,2,1); imshow(data.img); 
subplot(1,2,2); 
% C = -cam_R'*cam_T;
% scatter3(C(1), C(2), C(3));

% T^b_c translation from frame c to frame b (w.r.t. frame b)
Tb_c = [0.02*sqrt(2); -0.02*sqrt(2); -0.03];
% R^b_c rotation from frame c to frame b (w.r.t. frame b)
Rb_c = [sqrt(2)/2, -sqrt(2)/2, 0; -sqrt(2)/2, -sqrt(2)/2, 0; 0, 0, -1];

[Rw_b, Tw_b] = get_body_pose(cam_R, cam_T, Rb_c, Tb_c);
scatter3(Tw_b(1), Tw_b(2), Tw_b(3));
hold on;
scatter3(vicon_v(1), vicon_v(2), vicon_v(3));
plotCamera('Location', (-cam_R'*cam_T)' ,'Orientation', cam_R', 'Opacity',0, 'Size', 0.1);
scatter3(pts(1,:), pts(2,:), pts(3,:));
rectangle('Position',[0 0 3.4960 2.6360]);
xlim([-1, 3.5]);
ylim([-1, 3]);
zlim([0, 3]);
xlabel('x');
ylabel('y');
zlabel('z');
hold off; 
drawnow;


