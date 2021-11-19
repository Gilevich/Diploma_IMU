close all;
clear;
clc;

filename = 'RealStaticData.log';
%filename = 'teraterm.log';
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
Ay = -(180/pi) * atan(Accelerometer(:,1)./(sqrt(Accelerometer(:,2).^2 + Accelerometer(:,3).^2)));
Ax = (180/pi) * atan(Accelerometer(:,2)./(sqrt(Accelerometer(:,1).^2 + Accelerometer(:,3).^2)));
Az = (180/pi) * atan(Accelerometer(:,3)./(sqrt(Accelerometer(:,1).^2 + Accelerometer(:,2).^2)));

% %%%%%%%%%%%%%%%%%%%%%%% Complementary filtr %%%%%%%%%%%%%%%%%%%%%%%%%%

K = 0.9992;

Alpha = zeros(size(Gyroscope(:,1)));
Beta = zeros(size(Gyroscope(:,2)));
Gamma = zeros(size(Gyroscope(:,3)));

Alpha_K = zeros(size(Gyroscope(:,1)));
Beta_K = zeros(size(Gyroscope(:,2)));
Gamma_K = zeros(size(Gyroscope(:,3)));

for t = 1:length(time)
    if t == 1
        Alpha(t) = Alpha(t) * dt;
        Beta(t) = Beta(t) * dt;
        Gamma(t) = Gamma(t) * dt;
    else
        Alpha(t) = Alpha(t-1) + Gyroscope(t,1) * dt;
        Beta(t) = Beta(t-1) + Gyroscope(t,2) * dt;
        Gamma(t) = Gamma(t-1) + Gyroscope(t,3) * dt;
    end
end
% 
% for t = 1:length(time)
%     if t == 1
%         Alpha_K(t) = K * (Alpha_K(t) * dt) + (1-K) * Ax(t);
%         Beta_K(t) = K * (Beta_K(t) * dt) + (1-K) * Ay(t);
%         Gamma_K(t) = K * (Gamma_K(t) * dt) + (1-K) * Az(t);
%     else
%         Alpha_K(t) = K * (Alpha_K(t-1) + Gyroscope(t,1) * dt) + (1-K) * Ax(t);
%         Beta_K(t) = K * (Beta_K(t-1) + Gyroscope(t,2) * dt) + (1-K) * Ay(t);
%         Gamma_K(t) = K * (Gamma_K(t-1) + Gyroscope(t,3) * dt) + (1-K) * Az(t);
%     end
% end
% 
% figure('Name', 'Filtr Komplementarny');
% hold on;
% plot(time, Ax, 'r');
% plot(time, Alpha, 'g');
% plot(time, Alpha_K, 'b');
% title('Oś X');
% xlabel('Czas (s)');
% ylabel('Kąt (deg)');
% legend('Acc_X', 'Gyr_X', 'Out_X');
% hold off;
% 
% figure('Name', 'Filtr Komplementarny');
% hold on;
% plot(time, Ay, 'r');
% plot(time, Beta, 'g');
% plot(time, Beta_K, 'b');
% title('Oś Y');
% xlabel('Czas (s)');
% ylabel('Kąt (deg)');
% legend('Acc_Y', 'Gyr_Y', 'Out_Y');
% hold off;
% 
% figure('Name', 'Filtr Komplementarny');
% hold on;
% plot(time, Az, 'r');
% plot(time, Gamma, 'g');
% plot(time, Gamma_K, 'b');
% title('Oś Z');
% xlabel('Czas (s)');
% ylabel('Kąt (deg)');
% legend('Acc_Z', 'Gyr_Z', 'Out_Z');
% hold off;
% 
% figure('Name', 'Filtr Komplementarny');
% hold on;
% plot(time, Alpha_K, 'r');
% plot(time, Beta_K, 'g');
% plot(time, Gamma_K, 'b');
% title('Oś X, Y, Z');
% xlabel('Time (s)');
% ylabel('Angle (deg)');
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
% plot(time, Ax, 'r');
% plot(time, Alpha, 'g');
% plot(time, euler(:,1), 'b');
% title('Oś X');
% xlabel('Czas (s)');
% ylabel('Kąt (deg)');
% legend('Acc_X', 'Gyr_X', 'Out_X');
% hold off;
% 
% euler = quatern2euler(quaternConj(quaternion)) * (180/pi);	
% figure('Name', 'Filtr Madgwicka');
% hold on;
% plot(time, Ay, 'r');
% plot(time, Beta, 'g');
% plot(time, euler(:,2), 'b');
% title('Oś Y');
% xlabel('Czas (s)');
% ylabel('Kąt (deg)');
% legend('Acc_Y', 'Gyr_Y', 'Out_Y');
% hold off;
% 
% euler = quatern2euler(quaternConj(quaternion)) * (180/pi);	
% figure('Name', 'Filtr Madgwicka');
% hold on;
% plot(time, Az, 'r');
% plot(time, Gamma, 'g');
% plot(time, euler(:,3), 'b');
% title('Oś Z');
% xlabel('Czas (s)');
% ylabel('Kąt (deg)');
% legend('Acc_Z', 'Gyr_Z', 'Out_Z');
% hold off;
% 
% figure('Name', 'Filtr Madgwicka');
% hold on;
% plot(time, euler(:,1), 'r');
% plot(time, euler(:,2), 'g');
% plot(time, euler(:,3), 'b');
% title('Oś X, Y, Z');
% xlabel('Time (s)');
% ylabel('Angle (deg)');
% legend('Alpha', 'Beta', 'Gamma');
% hold off;

%%%%%%%%%%%%%%%%%%%%%% Kalman filtr %%%%%%%%%%%%%%%%%%%%%%%%%%

W_alpha = Gyroscope(:,1);
W_beta = Gyroscope(:,2);
W_gamma = Gyroscope(:,3);

Alpha_Kalman = zeros(size(Gyroscope(:,1)));
Beta_Kalman = zeros(size(Gyroscope(:,2)));
Gamma_Kalman = zeros(size(Gyroscope(:,3)));

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

q = 0.000005;
r = 1000;

Q = eye(6).* q; %macierz kowariancji modelu
R = eye(3).* r; %macierz kowariancji pomiarow


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
Alpha_Kalman(k) = x_post(1);
Beta_Kalman(k) = x_post(2);
Gamma_Kalman(k) = x_post(3);
end

figure('Name', 'Filtr Kalmana');
hold on;
plot(time, Ax, 'r');
plot(time, Beta, 'g');
plot(time, Alpha_Kalman, 'b');
title('Oś X');
ylim([-10 70]);
xlabel('Czas (s)');
ylabel('Kąt (deg)');
legend('Acc_X', 'Gyr_X', 'Out_X');
hold off;

figure('Name', 'Filtr Kalmana');
hold on;
plot(time, Ay, 'r');
plot(time, Beta, 'g');
plot(time, Beta_Kalman, 'b');
title('Oś Y');
ylim([-10 70]);
xlabel('Czas (s)');
ylabel('Kąt (deg)');
legend('Acc_Y', 'Gyr_Y', 'Out_Y');
hold off;

figure('Name', 'Filtr Kalmana');
hold on;
plot(time, Az, 'r');
plot(time, Gamma, 'g');
plot(time, Gamma_Kalman, 'b');
title('Oś Z');
ylim([-10 140]);
xlabel('Czas (s)');
ylabel('Kąt (deg)');
legend('Acc_Z', 'Gyr_Z', 'Out_Z');
hold off;

figure('Name', 'Filtr Kalmana');
hold on;
plot(time, Alpha_Kalman, 'r');
plot(time, Beta_Kalman, 'g');
plot(time, Gamma_Kalman, 'b');
title('Oś X, Y, Z');
ylim([-10 100]);
xlabel('Time (s)');
ylabel('Angle (deg)');
legend('Alpha', 'Beta', 'Gamma');
hold off;