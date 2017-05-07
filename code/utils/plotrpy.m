function plotrpy(rpy, rpy_vicon, ts, n_tags)

start = 200;
rpy = rpy(:, start:end);
rpy_vicon = rpy_vicon(:, start:end);
ts = ts(start:end);
n_tags = n_tags(start:end);

figure;
err_norm = sum((rpy-rpy_vicon).^2)/3;
subplot(5,1,1);
plot(ts, err_norm);
title('norm of errors');
disp 'avg norm of rpy error:';
sum(err_norm)/numel(ts)

sum((rpy-rpy_vicon).^2,2)/numel(ts)


subplot(5,1,2);
plot(ts, n_tags);
title('number of tags');



subplot(5,1,3); 
plot(ts, rpy(1,:));
hold on;
plot(ts, rpy_vicon(1,:));
hold off;
title('roll');

legend('estimated', 'vicon');

subplot(5,1,4);
plot(ts, rpy(2,:));
hold on;
plot(ts, rpy_vicon(2,:));
hold off;
title('pitch');

legend('estimated', 'vicon');

subplot(5,1,5);
plot(ts, rpy(3,:));
hold on;
plot(ts, rpy_vicon(3,:));
hold off;
title('yaw');

legend('estimated', 'vicon');

end