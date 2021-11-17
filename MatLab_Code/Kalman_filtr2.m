%close all;
clear;
clc;

load('ExampleData.mat')

% Ax = atan(Accelerometer(:,1)./(sqrt(Accelerometer(:,2).^2 + Accelerometer(:,3).^2)));
% Ay = atan(Accelerometer(:,2)./(sqrt(Accelerometer(:,1).^2 + Accelerometer(:,3).^2)));
% Az = atan(Accelerometer(:,3)./(sqrt(Accelerometer(:,1).^2 + Accelerometer(:,2).^2)));

Ax = (180/pi) * atan(Accelerometer(:,1)./(sqrt(Accelerometer(:,2).^2 + Accelerometer(:,3).^2)));
Ay = (180/pi) * atan(Accelerometer(:,2)./(sqrt(Accelerometer(:,1).^2 + Accelerometer(:,3).^2)));
Az = (180/pi) * atan(Accelerometer(:,3)./(sqrt(Accelerometer(:,1).^2 + Accelerometer(:,2).^2)));

W_alpha = Gyroscope(:,1);
W_beta = Gyroscope(:,2);
W_gamma = Gyroscope(:,3);

Alpha = zeros(size(Gyroscope(:,1)));
Beta = zeros(size(Gyroscope(:,2)));
Gamma = zeros(size(Gyroscope(:,3)));
dt = 1/256;

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

q = 0.00001;
r = 8500;
p = 0;
Q = eye(6).* q; %macierz kowariancji modelu
R = eye(3).* r; %macierz kowariancji pomiarow
P = eye(6).* p; %macierz kowariancji stanu

x_post = zeros(6,1);
P_post = zeros(6,6);


for k = 2:length(time)-1

%Predykcja
x_pri = A * x_post + B * u(:,k-1); 
P_pri = A * P_post .* A' + Q;
%Korekcja
K = P_pri * H' * (H * P_pri * H' + R)^(-1);
x_post = x_pri + K * (z(:,k) - (H * x_pri));
P_post = (I - K * H) * P_pri;

%zapis
Alpha(k) = x_post(1);
Beta(k) = x_post(2);
Gamma(k) = x_post(3);
end


Kalman_filer_function(z(:,1),u(:,0),x_post, P_post);

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