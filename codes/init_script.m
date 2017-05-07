% Add additional inputs after the given ones if you want to
% Example:
% your_input = 1;
% ekf_handle1 = @(sensor, vic) eskf1(sensor, vic, your_input);
% ekf_handle2 = @(sensor) eskf2(sensor, your_input);
%
% We will only call ekf_handle in the test function.
% Note that this will only create a function handle, but not run the function

ekf1_handle = @(sensor, vic) ekf1(sensor, vic);
ekf2_handle = @(sensor) ekf2(sensor);
