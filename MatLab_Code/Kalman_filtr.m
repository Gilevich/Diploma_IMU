%close all;
clear;
clc;

load('ExampleData.mat');
% figure('Name', 'Dane syntetyczne');
% axis(1) = subplot(2,1,1);
% hold on;
% plot(time, Gyroscope(:,1), 'r');
% plot(time, Gyroscope(:,2), 'g');
% plot(time, Gyroscope(:,3), 'b');
% legend('X', 'Y', 'Z');
% xlabel('Time (s)');
% ylabel('Angular rate (deg/s)');
% title('Gyroscope');
% hold off;
% axis(2) = subplot(2,1,2);
% hold on;
% plot(time, Accelerometer(:,1), 'r');
% plot(time, Accelerometer(:,2), 'g');
% plot(time, Accelerometer(:,3), 'b');
% legend('X', 'Y', 'Z');
% xlabel('Time (s)');
% ylabel('Acceleration (g)');
% title('Accelerometer');
% hold off;

Ax = atan(Accelerometer(:,1)./(sqrt(Accelerometer(:,2).^2 + Accelerometer(:,3).^2)));
Ay = atan(Accelerometer(:,2)./(sqrt(Accelerometer(:,1).^2 + Accelerometer(:,3).^2)));
Az = atan(Accelerometer(:,3)./(sqrt(Accelerometer(:,1).^2 + Accelerometer(:,2).^2)));

Alpha = zeros(size(Gyroscope(:,1)));
Beta = zeros(size(Gyroscope(:,2)));
Gamma = zeros(size(Gyroscope(:,3)));
P = zeros(size(time));

sigma1 = 100; %gyroscope error
sigma2 = 130; %acc error


for t = 1:length(time)
    if t == 1
        P(t) = sigma1^2;
        Alpha(t) = Gyroscope(t,1);
        Beta(t) = Gyroscope(t,2);
        Gamma(t) = Gyroscope(t,3);
        
        P(t) = (P(t) * sigma2^2) / (P(t)+sigma2^2);
        Alpha(t) = Alpha(t) + (P(t)/sigma2^2)*(Ax(t)-Alpha(t));
        Beta(t) = Beta(t) + (P(t)/sigma2^2)*(Ay(t)-Beta(t));
        Gamma(t) = Gamma(t) + (P(t)/sigma2^2)*(Az(t)-Gamma(t));
    else
        P(t) = (P(t-1) * sigma1^2) / (P(t-1)+sigma1^2);
        Alpha(t) = Alpha(t-1) + (P(t)/sigma1^2)*(Gyroscope(t,1)-Alpha(t-1));
        Beta(t) = Beta(t-1) + (P(t)/sigma1^2)*(Gyroscope(t,2)-Beta(t-1));
        Gamma(t) = Gamma(t-1) + (P(t)/sigma1^2)*(Gyroscope(t,3)-Gamma(t-1));
        
        P(t) = (P(t) * sigma2^2) / (P(t)+sigma2^2);
        Alpha(t) = Alpha(t) + (P(t)/sigma2^2)*(Ax(t)-Alpha(t));
        Beta(t) = Beta(t) + (P(t)/sigma2^2)*(Ay(t)-Beta(t));
        Gamma(t) = Gamma(t) + (P(t)/sigma2^2)*(Az(t)-Gamma(t));
    end
end

figure('Name', 'Filtr Kalmana');
hold on;
plot(time, Alpha, 'r');
plot(time, Beta, 'g');
plot(time, Gamma, 'b');
title('Filtr Kalmana');
xlabel('Time (s)');
ylabel('Angle (deg)');
legend('Alpha', 'Beta', 'Gamma');
hold off;