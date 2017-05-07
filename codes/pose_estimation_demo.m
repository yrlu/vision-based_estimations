%% 
addpath ../data;
addpath ../quat;
addpath ./utils;

%% check data format
load('data/studentdata4.mat');

%% Constants
% T^b_c translation from frame c to frame b (w.r.t. frame b)
Tb_c = [0.02*sqrt(2); -0.02*sqrt(2); -0.03];
% R^b_c rotation from frame c to frame b (w.r.t. frame b)
Rb_c = [sqrt(2)/2, -sqrt(2)/2, 0; -sqrt(2)/2, -sqrt(2)/2, 0; 0, 0, -1];
%
K = [311.0520 0 201.8724; 0 311.3885 113.6210; 0 0 1];

%% Run code against the dataset

OUTPUT_TO_VIDEO = 0;
DISPLAY_EVERY = 8;

if OUTPUT_TO_VIDEO == 1
    v = VideoWriter('pose_estimation.avi');
    open(v)
end

tic
rpy = zeros(3, numel(data));
rpy_vicon = zeros(3, numel(data));
ts = zeros(1, numel(data));
n_tags = zeros(1, numel(data));

xyz = zeros(3, numel(data));
xyz_vicon = zeros(3, numel(data));
for i = 1:numel(data)
    n_tags(i) = numel(data(i).id);
    if(numel(data(i).id)>0)
        [Tw_b, q] = estimate_pose(data(i));
        Rw_b = quat2matrix(q);
        vicon_v = get_vicon(data(i), vicon, time);
        if mod(i, DISPLAY_EVERY) == 0
            plot_data4(data(i), Rw_b, Tw_b, vicon_v);
            if OUTPUT_TO_VIDEO == 1
                im = frame2im(getframe(gcf));
                writeVideo(v,im);
            end
        end
        [r,p,y]=rot2rpy(Rw_b);
        rpy(:,i) = [r,p,y]';
        rpy_vicon(:,i) = vicon_v(4:6);
        
        xyz(:,i) = Tw_b;
        xyz_vicon(:,i) = vicon_v(1:3);
    end
    ts(i) = data(i).t;
end
toc

if OUTPUT_TO_VIDEO == 1
    close(v);
end
%% plot estimation vs ground truth
plotrpy(rpy, rpy_vicon, ts, n_tags);
plotpos(xyz, xyz_vicon, ts, n_tags);
