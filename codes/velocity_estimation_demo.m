%% 
clear;
addpath ../data;
addpath ../quat;
addpath ./utils;

%% check data format
load('data/studentdata4.mat');

%%
vels = zeros(3, numel(data));
omgs = zeros(3, numel(data));

vicon_vels = zeros(3, numel(data));
vicon_omgs = zeros(3, numel(data));
ts = zeros(1,numel(data));

pos = zeros(3, numel(data));
vicon_pos = zeros(3, numel(data));
%%
tic
for i = 1:numel(data)
    if mod(i,200) == 0
        disp(int2str(i))
    end
    if numel(data(i).id)
        p = estimate_pose(data(i));
        [vel, omg] = estimate_vel(data(i));
        vicon_v = get_vicon(data(i), vicon, time);
        vels(:, i) = vel;
        omgs(:, i) = omg;
        pos(:, i) = p;
        vicon_vels(:, i) = vicon_v(7:9);
        vicon_omgs(:, i) = vicon_v(10:end);
        vicon_pos(:,i) = vicon_v(1:3);
    end
    ts(i) = data(i).t;
end
toc

plot_vel_omg(vels, omgs, vicon_vels, vicon_omgs, ts);
