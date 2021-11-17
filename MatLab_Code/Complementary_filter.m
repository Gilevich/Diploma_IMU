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
xlabel('Czas (s)');
ylabel('Prędkość kątowa(deg/s)');
title('Żyroskop');
hold off;
axis(2) = subplot(2,1,2);
hold on;
plot(time, Accelerometer(:,1), 'r');
plot(time, Accelerometer(:,2), 'g');
plot(time, Accelerometer(:,3), 'b');
legend('X', 'Y', 'Z');
xlabel('Czas (s)');
ylabel('Przyspieszenie (g)');
title('Akcelerometr');
hold off;

K = 0.9998;

Ax = (180/pi) * atan(Accelerometer(:,1)./(sqrt(Accelerometer(:,2).^2 + Accelerometer(:,3).^2)));
Ay = (180/pi) * atan(Accelerometer(:,2)./(sqrt(Accelerometer(:,1).^2 + Accelerometer(:,3).^2)));
Az = (180/pi) * atan(Accelerometer(:,3)./(sqrt(Accelerometer(:,1).^2 + Accelerometer(:,2).^2)));

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

figure('Name', 'Filtr Komplementarny');
hold on;
plot(time, Alpha, 'r');
plot(time, Beta, 'g');
plot(time, Gamma, 'b');
title('Filtr Komplementarny');
xlabel('Czas (s)');
ylabel('Kąt (deg)');
legend('Alpha', 'Beta', 'Gamma');
hold off;