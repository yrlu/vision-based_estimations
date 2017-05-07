function plotpos(xyz, xyz_vicon, ts, n_tags)
disp 'plot positions';
figure(1);

start = 200;
xyz = xyz(:, start:end);
xyz_vicon = xyz_vicon(:, start:end);
ts = ts(start:end);
n_tags = n_tags(start:end);

err_norm = sum((xyz-xyz_vicon).^2)/3;
subplot(5,1,1);
plot(ts, err_norm);
title('norm of errors');
disp 'avg norm of pos error:';
sum(err_norm)/numel(ts)

sum((xyz-xyz_vicon).^2,2)/numel(ts)


subplot(5,1,2);
plot(ts, n_tags);
title('number of tags');


subplot(5,1,3); 
plot(ts, xyz(1,:));
hold on;
plot(ts, xyz_vicon(1,:));
hold off;
title('x');

legend('estimated', 'vicon');

subplot(5,1,4);
plot(ts, xyz(2,:));
hold on;
plot(ts, xyz_vicon(2,:));
hold off;
title('y');

legend('estimated', 'vicon');

subplot(5,1,5);
plot(ts, xyz(3,:));
hold on;
plot(ts, xyz_vicon(3,:));
hold off;
title('z');

legend('estimated', 'vicon');

end