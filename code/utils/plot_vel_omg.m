% function plot_vel_omg(vels, omgs, ts, vicon, time)
function plot_vel_omg(vels, omgs, vicon_vels, vicon_omgs, ts)
disp 'plot velocities';
start = 1;
figure(1); 
subplot(4,1,1); 
plot(ts(start:end), vels(1,start:end));
hold on;
plot(ts(start:end), vicon_vels(1,start:end));
hold off;
title('vx')
legend('estimated', 'vicon');

subplot(4,1,2);
plot(ts(start:end), vels(2,start:end));
hold on;
plot(ts(start:end), vicon_vels(2,start:end));
hold off;
title('vy');
legend('estimated', 'vicon');

subplot(4,1,3);
plot(ts(start:end), vels(3,start:end));
hold on;
plot(ts(start:end), vicon_vels(3,start:end));
hold off;
title('vz');
legend('estimated', 'vicon');

subplot(4,1,4);
mse = mean((vels - vicon_vels).*2);
plot(ts(start:end), mse(start:end));
title('mean square error');
% sum(mean((vels - vicon_vels).^2))/size(vicon_vels, 2)
vel_err_xyz = sum((vels - vicon_vels).^2,2)/size(vicon_vels, 2);
vel_err = mean(sum((vels - vicon_vels).^2,2)/size(vicon_vels, 2));


figure(2);
subplot(4,1,1);
plot(ts(start:end), omgs(1,start:end));
hold on;
plot(ts(start:end), vicon_omgs(1,start:end));
hold off;
title('angular vx');
legend('estimated', 'vicon');

subplot(4,1,2);
plot(ts(start:end), omgs(2,start:end));
hold on;
plot(ts(start:end), vicon_omgs(2,start:end));
hold off;
title('angular vy');
legend('estimated', 'vicon');

subplot(4,1,3);
plot(ts(start:end), omgs(3,start:end));
hold on;
plot(ts(start:end), vicon_omgs(3,start:end));
hold off;
title('angular vz');
legend('estimated', 'vicon');

subplot(4,1,4);
mse = mean((omgs - vicon_omgs).*2);
plot(ts(start:end), mse(start:end));
title('mean square error');
omg_err_xyz = sum((omgs - vicon_omgs).^2,2)/size(vicon_omgs, 2);
omg_err = mean(sum((omgs - vicon_omgs).^2,2)/size(vicon_omgs, 2));

[vel_err_xyz, omg_err_xyz]
[vel_err, omg_err]

end