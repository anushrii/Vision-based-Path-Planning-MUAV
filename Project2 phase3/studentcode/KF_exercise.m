close all;
clear;
clc;

initialize;

%% Simulate the system using ODE45
[Ts, XS] = ode45(@(t,x) noisy_model(t, x, A, B, sgm, bias), Ts, x0); % Solve ODE
XS = XS';



%% Simualate the sensors

for i=1:4 
XSMax(i)=max(XS(i,:));
end
ZS(1,:)=XS(1,:)+0.05*randn(size(Ts'))*XSMax(1);
ZS(2,:)=XS(2,:)+0.1*randn(size(Ts'))*XSMax(2);
ZS(3,:)=XS(3,:)+0.15*randn(size(Ts'))*XSMax(3);
ZS(4,:)=XS(4,:)+0.2*randn(size(Ts'))*XSMax(4);


% ZS(2,1) = 0;
% ZS(2,2:end) =(ZS(1,2:end) - ZS(1,1:end-1)).*dt;
% 
% ZS(3,1:2) = 0;
% ZS(3,3:end) = (ZS(2,3:end) - ZS(2,2:end-1)).*dt;
% 
% ZS(4,1:3) = 0;
% ZS(4,4:end) = (ZS(3,4:end) - ZS(3,3:end-1)).*dt;
% 
% XS_1 = zeros(size(XS));
% XS_1(2,1) = 0;
% XS_1(2,2:end) =(XS(1,2:end) - XS(1,1:end-1)).*dt;
% 
% XS_1(3,1:2) = 0;
% XS_1(3,3:end) = (XS_1(2,3:end) - XS_1(2,2:end-1)).*dt;
% 
% XS_1(4,1:3) = 0;
% XS_1(4,4:end) = (XS_1(3,4:end) - XS_1(3,3:end-1)).*dt;


n =2;
% [filteredOutput] = mykalman(XS, ZS, Ts);
 [filteredOutput1] = mykalman1(XS, ZS, Ts,n);
filteredOutput =filteredOutput1';

%% Figures
figure; hold on; grid on;
plot(Ts, XS(1, :), 'r');
plot(Ts, XS(2, :), 'g');
plot(Ts, XS(3, :), 'b');
plot(Ts, XS(4, :), 'c');
plot(Ts, input_fun(Ts), 'm');
h = legend('$x$', '$x^{(i)}$', '$x^{(ii)}$', '$x^{(iii)}$', 'u', 'Location', 'NorthWest');
set(h,'Interpreter','latex');
set(h,'FontSize', 16);
xlabel('Time (seconds)');
title('Model Simulation');


% figure; hold on; grid on;
% plot(Ts, XS(1, :), Ts, ZS(1, :), 'r');
% h = legend('$x$', '$z$','Location', 'NorthWest');
% set(h,'Interpreter','latex');
% set(h,'FontSize', 16);
% xlabel('Time (seconds)');
% title('Sensed Information (Position)');
% figure; hold on; grid on;
% plot(Ts, XS(2, :), Ts, ZS(2, :), 'b');
% h = legend('$\dot{x}$', '$\dot{z}$','Location', 'NorthWest');
% set(h,'Interpreter','latex');
% set(h,'FontSize', 16);
% xlabel('Time (seconds)');
% title('Sensed Information (Velocity)');
% figure; hold on; grid on;
% plot(Ts, XS(3, :), Ts, ZS(3, :), 'k');
% h = legend('$\ddot{x}$', '$\ddot{z}$','Location', 'NorthWest');
% set(h,'Interpreter','latex');
% set(h,'FontSize', 16);
% xlabel('Time (seconds)');
% title('Sensed Information (Acceleration)');
% figure; hold on; grid on;
% plot(Ts, XS(4, :), Ts, ZS(4, :), 'r');
% h = legend('${x}^{(3)}$', '${z}^{(3)}$','Location', 'NorthWest');
% set(h,'Interpreter','latex');
% set(h,'FontSize', 16);
% xlabel('Time (seconds)');
% title('Sensed Information (Jerk)');

figure; hold on; grid on;
% subplot(4,1,1)
plot(Ts, XS(1, :), Ts, filteredOutput(1, :), 'r');
h = legend('$x$', '$z$','Location', 'NorthWest');
set(h,'Interpreter','latex');
set(h,'FontSize', 16);
xlabel('Time (seconds)');
title('Sensed Information (Position)');
figure; hold on; grid on;
% subplot(4,1,2)
plot(Ts, XS(2, :), Ts, filteredOutput(2, :), 'b');
h = legend('$\dot{x}$', '$\dot{z}$','Location', 'NorthWest');
set(h,'Interpreter','latex');
set(h,'FontSize', 16);
xlabel('Time (seconds)');
title('Sensed Information (Velocity)');
figure; hold on; grid on;
% subplot(4,1,3)
plot(Ts, XS(3, :), Ts, filteredOutput(3, :), 'k');
h = legend('$\ddot{x}$', '$\ddot{z}$','Location', 'NorthWest');
set(h,'Interpreter','latex');
set(h,'FontSize', 16);
xlabel('Time (seconds)');
title('Sensed Information (Acceleration)');
figure; hold on; grid on;
% subplot(4,1,4)
plot(Ts, XS(4, :), Ts, filteredOutput(4, :), 'r');
h = legend('${x}^{(3)}$', '${z}^{(3)}$','Location', 'NorthWest');
set(h,'Interpreter','latex');
set(h,'FontSize', 16);
xlabel('Time (seconds)');
title('Sensed Information (Jerk)');
