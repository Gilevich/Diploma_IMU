close all;
clear;
clc;


n = 25;
dt = 0.1;
t = 0:dt:n;
w = zeros(size(t,2),3);
g_bias = 0;

for i = 1:3
for j = 1:size(t,2)   
    if i == 1
        if j < 20 || j > 70
            w(j,i) = 10*randn(1);
        else
            w(j,i) = 10*randn(1)+20;
        end
    
    elseif i == 2
        if j < 90 || j > 140
            w(j,i) = 10*randn(1);
        else
            w(j,i) = 10*randn(1)+20;
        end
    
    elseif i == 3
        if j < 160 || j > 210
            w(j,i) = 10*randn(1);
        else
            w(j,i) = 10*randn(1)+20;
        end
    end
end
end

figure('Name', 'W');
hold on;
plot(t, w(:,1), 'r');
plot(t, w(:,2), 'g');
plot(t, w(:,3), 'b');
title('Filtr Complementarny');
xlabel('Time (s)');
ylabel('Angle (deg)');
legend('Alpha', 'Beta', 'Gamma');
hold off;

teta = zeros(size(t,2),3);
for i = 1:3
    for j = 1:size(t,2)
        if j == 1
            teta(j,i) = (w(j,i)-g_bias)*dt;
        else
            teta(j,i) = teta(j-1,i) + (w(j,i)-g_bias)*dt;
        end
    end
end

figure('Name', 'Teta');
hold on;
plot(t, teta(:,1), 'r');
plot(t, teta(:,2), 'g');
plot(t, teta(:,3), 'b');
title('Filtr Complementarny');
xlabel('Time (s)');
ylabel('Angle (deg)');
legend('Alpha', 'Beta', 'Gamma');
hold off;