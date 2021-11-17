function [x_post_data, p_post_data] = Kalman_filer_function(acc_data,gyr_data, old_x_post_data, old_p_post_data)
I = eye(6);
dt = 1/256;
acc_data_ = acc_data';
gyr_data_ = gyr_data';
%x(k) = A * x(k-1) + B * u(k-1) + W(k-1) - model procesu
%x = [x y z g_bias_x g_bias_y g_bias_z]'; %wektor stanu 6x1
%x = zeros(6,length(time));
A = [1 0 0 -dt 0 0;
     0 1 0 0 -dt 0;
     0 0 1 0 0 -dt;
     0 0 0 1 0  0;
     0 0 0 0 1  0;
     0 0 0 0 0  1;]; %macierz stanu 6x6
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


q = 0.00001;
r = 8500;
p = 0;
Q = eye(6).* q; %macierz kowariancji modelu
R = eye(3).* r; %macierz kowariancji pomiarow
P = eye(6).* p; %macierz kowariancji stanu

%Predykcja
x_pri = A * old_x_post_data + B * gyr_data_; 
P_pri = A .* old_p_post_data .* A' + Q;
%Korekcja
K = P_pri * H' * (H * P_pri * H' + R)^(-1);
x_post_data = x_pri + K * (acc_data_ - (H * x_pri));
p_post_data = (I - K * H) * P_pri;

end

