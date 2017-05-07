function [Rc_w, Tc_w] = estimate_cam_pose(sensor, inv_K)
if(numel(sensor.id)==0)
    Rc_w = eye(3);
    Tc_w = zeros(3,1);
    return
end
pts_w = [get_tag_coords2(sensor.id); zeros(1, numel(sensor.id)*5)]; 
pts_c = [sensor.p0, sensor.p1, sensor.p2, sensor.p3, sensor.p4];
[Rc_w, Tc_w] = estimate_cam_RT(pts_c, pts_w, inv_K);
end