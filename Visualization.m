clear; close all; clc;

load('Ant1_one_antennas1.mat')
alpha = double(alpha);
figure
hold on
plot(alpha)
plot(filtered_data)
hold off
grid on
xlim([1 length(filtered_data)])
legend('AoA without kalman filter','AoA with kalman filter')
xlabel('t, sample')
ylabel('AoA        ','rotation',0)

disp(['Дисперсия без фильтра = ' num2str(var(alpha))])
disp(['Дисперсия с фильтром = ' num2str(var(filtered_data))])