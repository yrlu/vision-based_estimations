function [vicon_v] = get_vicon(data, vicon, vicon_ts)
[~, id] = min(abs(vicon_ts - data.t));
vicon_v = vicon(:, id);