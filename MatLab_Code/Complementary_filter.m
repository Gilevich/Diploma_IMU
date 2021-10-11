close all;
clear;
clc;

load('ExampleData.mat');
figure('Name', 'Dane syntetyczne');
axis(1) = subplot(2,1,1);
hold on;
plot(time, Gyroscope(:,1), 'r');
plot(time, Gyroscope(:,2), 'g');
plot(time, Gyroscope(:,3), 'b');
legend('X', 'Y', 'Z');
xlabel('Time (s)');
ylabel('Angular rate (deg/s)');
title('Gyroscope');
hold off;
axis(2) = subplot(2,1,2);
hold on;
plot(time, Accelerometer(:,1), 'r');
plot(time, Accelerometer(:,2), 'g');
plot(time, Accelerometer(:,3), 'b');
legend('X', 'Y', 'Z');
xlabel('Time (s)');
ylabel('Acceleration (g)');
title('Accelerometer');
hold off;

K = 0.995;

Ax = atan(Accelerometer(:,1)./(sqrt(Accelerometer(:,2).^2 + Accelerometer(:,3).^2)));
Ay = atan(Accelerometer(:,2)./(sqrt(Accelerometer(:,1).^2 + Accelerometer(:,3).^2)));
Az = atan(Accelerometer(:,3)./(sqrt(Accelerometer(:,1).^2 + Accelerometer(:,2).^2)));

Alpha = zeros(size(Gyroscope(:,1)));
Beta = zeros(size(Gyroscope(:,2)));
Gamma = zeros(size(Gyroscope(:,3)));
dt = 1/256;

for t = 1:length(time)
    if t == 1
        Alpha(t) = K * (Alpha(t) * dt) + (1-K) * Ax(t);
        Beta(t) = K * (Beta(t) * dt) + (1-K) * Ay(t);
        Gamma(t) = K * (Gamma(t) * dt) + (1-K) * Az(t);
    else
        Alpha(t) = K * (Alpha(t-1) + Gyroscope(t,1) * dt) + (1-K) * Ax(t);
        Beta(t) = K * (Beta(t-1) + Gyroscope(t,2) * dt) + (1-K) * Ay(t);
        Gamma(t) = K * (Gamma(t-1) + Gyroscope(t,3) * dt) + (1-K) * Az(t);
    end
end

figure('Name', 'Filtr Complementarny K=0.995');
hold on;
plot(time, Alpha, 'r');
plot(time, Beta, 'g');
plot(time, Gamma, 'b');
title('Filtr Complementarny');
xlabel('Time (s)');
ylabel('Angle (deg)');
legend('Alpha', 'Beta', 'Gamma');
hold off;