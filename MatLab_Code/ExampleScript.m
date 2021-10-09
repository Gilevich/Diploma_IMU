
%% Start of script

addpath('quaternion_library');      % include quaternion library
%%close all;                          % close all figures
clear;                              % clear all variables
clc;                                % clear the command terminal

%% Import and plot sensor data

load('ExampleData.mat');

figure('Name', 'Sensor Data');
axis(1) = subplot(3,1,1);
hold on;
plot(time, Gyroscope(:,1), 'r');
plot(time, Gyroscope(:,2), 'g');
plot(time, Gyroscope(:,3), 'b');
legend('X', 'Y', 'Z');
xlabel('Time (s)');
ylabel('Angular rate (deg/s)');
title('Gyroscope');
hold off;
axis(2) = subplot(3,1,2);
hold on;
plot(time, Accelerometer(:,1), 'r');
plot(time, Accelerometer(:,2), 'g');
plot(time, Accelerometer(:,3), 'b');
legend('X', 'Y', 'Z');
xlabel('Time (s)');
ylabel('Acceleration (g)');
title('Accelerometer');
hold off;
linkaxes(axis, 'x');

%% Process sensor data through algorithm

AHRS = MadgwickAHRS('SamplePeriod', 1/256, 'Beta', 0.1);

quaternion = zeros(length(time), 4);
for t = 1:length(time)
    AHRS.UpdateIMU(Gyroscope(t,:) * (pi/180), Accelerometer(t,:));	% gyroscope units must be radians
    quaternion(t, :) = AHRS.Quaternion;
end

%% Plot algorithm output as Euler angles
% The first and third Euler angles in the sequence (phi and psi) become
% unreliable when the middle angles of the sequence (theta) approaches ±90
% degrees. This problem commonly referred to as Gimbal Lock.
% See: http://en.wikipedia.org/wiki/Gimbal_lock

euler = quatern2euler(quaternConj(quaternion)) * (180/pi);	% use conjugate for sensor frame relative to Earth and convert to degrees.

figure('Name', 'Filtr Madgwicka');
hold on;
plot(time, euler(:,1), 'r');
plot(time, euler(:,2), 'g');
plot(time, euler(:,3), 'b');
title('Filtr Madgwicka');
xlabel('Time (s)');
ylabel('Angle (deg)');
legend('Alpha', 'Beta', 'Gamma');
hold off;

%% End of script