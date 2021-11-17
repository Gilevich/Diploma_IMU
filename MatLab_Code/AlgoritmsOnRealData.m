close all;
clear;
clc;

%filename = 'RealStaticData.log';
filename = 'teraterm.log';
delimiterIn = '\t';
headerlinesIn = 1;
RealData = importdata(filename,delimiterIn,headerlinesIn);
NewData = RealData.data/1000;
Accelerometer = NewData(:,1:3);
Gyroscope = NewData(:,4:6);

dt = 1/256;
time = zeros(1,length(RealData.data(:,1)));
time(1) = 0;

for i = 2:length(RealData.data(:,1))
    time(i) = time(i-1) + dt;
end

% figure('Name', 'Dane z czujnika');
% axis(1) = subplot(2,1,1);
% hold on;
% plot(time, Gyroscope(:,1), 'r');
% plot(time, Gyroscope(:,2), 'g');
% plot(time, Gyroscope(:,3), 'b');
% legend('X', 'Y', 'Z');
% xlabel('Czas (s)');
% ylabel('Prędkość kątowa(deg/s)');
% title('Żyroskop');
% hold off;
% axis(2) = subplot(2,1,2);
% hold on;
% plot(time, Accelerometer(:,1), 'r');
% plot(time, Accelerometer(:,2), 'g');
% plot(time, Accelerometer(:,3), 'b');
% legend('X', 'Y', 'Z');
% xlabel('Czas (s)');
% ylabel('Przyspieszenie (g)');
% title('Akcelerometr');
% hold off;
% 
Ax = (180/pi) * atan(Accelerometer(:,1)./(sqrt(Accelerometer(:,2).^2 + Accelerometer(:,3).^2)));
Ay = (180/pi) * atan(Accelerometer(:,2)./(sqrt(Accelerometer(:,1).^2 + Accelerometer(:,3).^2)));
Az = (180/pi) * atan(Accelerometer(:,3)./(sqrt(Accelerometer(:,1).^2 + Accelerometer(:,2).^2)));

% %%%%%%%%%%%%%%%%%%%%%%% Complementary filtr %%%%%%%%%%%%%%%%%%%%%%%%%%
% 
%  K = 0.99994;
% 
Alpha = zeros(size(Gyroscope(:,1)));
Beta = zeros(size(Gyroscope(:,2)));
Gamma = zeros(size(Gyroscope(:,3)));
% 
% for t = 1:length(time)
%     if t == 1
%         Alpha(t) = K * (Alpha(t) * dt) + (1-K) * Ax(t);
%         Beta(t) = K * (Beta(t) * dt) + (1-K) * Ay(t);
%         Gamma(t) = K * (Gamma(t) * dt) + (1-K) * Az(t);
%     else
%         Alpha(t) = K * (Alpha(t-1) + Gyroscope(t,1) * dt) + (1-K) * Ax(t);
%         Beta(t) = K * (Beta(t-1) + Gyroscope(t,2) * dt) + (1-K) * Ay(t);
%         Gamma(t) = K * (Gamma(t-1) + Gyroscope(t,3) * dt) + (1-K) * Az(t);
%     end
% end
% 
% figure('Name', 'Filtr Komplementarny');
% hold on;
% plot(time, Alpha, 'r');
% plot(time, Beta, 'g');
% plot(time, Gamma, 'b');
% title('Filtr Komplementarny');
% xlabel('Czas (s)');
% ylabel('Kąt (deg)');
% legend('Alpha', 'Beta', 'Gamma');
% hold off;
% 
% % %%%%%%%%%%%%%%%%%%%%%%% Madgwick filtr %%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% addpath('quaternion_library'); 
% AHRS = MadgwickAHRS('SamplePeriod', dt, 'Beta', 0.01);
% 
% quaternion = zeros(length(time), 4);
% for t = 1:length(time)
%     AHRS.UpdateIMU(Gyroscope(t,:) * (pi/180), Accelerometer(t,:));
%     quaternion(t, :) = AHRS.Quaternion;
% end
% 
% euler = quatern2euler(quaternConj(quaternion)) * (180/pi);	
% figure('Name', 'Filtr Madgwicka');
% hold on;
% plot(time, euler(:,1), 'r');
% plot(time, euler(:,2), 'g');
% plot(time, euler(:,3), 'b');
% title('Filtr Madgwicka');
% xlabel('Czas (s)');
% ylabel('Kąt (deg)');
% legend('Alpha', 'Beta', 'Gamma');
% hold off;

%%%%%%%%%%%%%%%%%%%%%%% Kalman filtr %%%%%%%%%%%%%%%%%%%%%%%%%%

W_alpha = Gyroscope(:,1);
W_beta = Gyroscope(:,2);
W_gamma = Gyroscope(:,3);

Alpha = zeros(size(Gyroscope(:,1)));
Beta = zeros(size(Gyroscope(:,2)));
Gamma = zeros(size(Gyroscope(:,3)));

I = eye(6);

%x(k) = A * x(k-1) + B * u(k-1) + W(k-1) - model procesu
%x = [x y z g_bias_x g_bias_y g_bias_z]'; %wektor stanu 6x1
x = zeros(6,length(time));
A = [1 0 0 -dt 0 0;
     0 1 0 0 -dt 0;
     0 0 1 0 0 -dt;
     0 0 0 1 0  0;
     0 0 0 0 1  0;
     0 0 0 0 0  1;]; %macierz stanu 6x6
u = [W_alpha W_beta W_gamma]';
B = [dt 0 0;
     0 dt 0;
     0 0 dt;
     0 0  0;
     0 0  0;
     0 0  0;]; %macierz wejscia(sterowania) 6x3
%z(k) = H * x(k) + V(k) - model pomiaru
H = [1 0 0 0 0 0;
     0 1 0 0 0 0;
     0 0 1 0 0 0;]; %macierz wyjscia 3x6

z = [Ax Ay Az]';

q = 0.000001;
r = 99990;

p = 0;
Q = eye(6).* q; %macierz kowariancji modelu
R = eye(3).* r; %macierz kowariancji pomiarow
P = eye(6).* p; %macierz kowariancji stanu

x_post = zeros(6,1);
P_post = zeros(6,6);


for k = 2:length(time)

%Predykcja
x_pri = A * x_post + B * u(:,k-1); 
P_pri = A * P_post * A' + Q;
%Korekcja
K = P_pri * H' * (H * P_pri * H' + R)^(-1);
x_post = x_pri + K * (z(:,k) - (H * x_pri));
P_post = (I - K * H) * P_pri;

%zapis
Alpha(k) = x_post(1);
Beta(k) = x_post(2);
Gamma(k) = x_post(3);
end

figure('Name', 'Filtr Kalmana');
hold on;
plot(time, Alpha, 'r');
plot(time, Beta, 'g');
plot(time, Gamma, 'b');
title('Filtr Kalmana');
xlabel('Czas (s)');
ylabel('Kąt (deg)');
legend('Alpha', 'Beta', 'Gamma');
hold off;