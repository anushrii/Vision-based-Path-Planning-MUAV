
%% main
%  addpath('./data')
% load('studentdata1.mat');
% load('studentdata4.mat');
% load('studentdata9.mat');
%% load data

clear all;
close all;
load data/studentdata9;
% 
init_script;

%% use vicon time points only

N = length(time);

% clear ekf1;
% clear ekf2;

% X = zeros(6,N);
X = zeros(9,N);
Z = zeros(6,N);
t = time;
tic
for i=1:N
    [~,si] = min(abs([data.t] - time(i)));
%     [X(:,i), Z(:,i)] = ekf1(data(si), struct('t', time(i), 'vel', vicon(7:12,i)),worldnew,func);
      [X(:,i), Z(:,i)] = ekf2(data(si),worldnew,func2);
end
toc



figure,
subplot(3,1,1)
plot(t, X(1,:))
hold on
plot(time,vicon(1,:),'r')

subplot(3,1,2)
plot(t,X(2,:))
hold on
plot(time,vicon(2,:),'r')

subplot(3,1,3)
plot(t, X(3,:))
hold on
plot(time,vicon(3,:),'r')

% figure,
% subplot(3,1,1)
% plot(t, X(4,:))
% hold on
% plot(time,vicon(4,:),'r')
% 
% subplot(3,1,2)
% plot(t,X(5,:))
% hold on
% plot(time,vicon(5,:),'r')
% 
% subplot(3,1,3)
% plot(t, X(6,:))
% hold on
% plot(time,vicon(6,:),'r')

figure,
subplot(3,1,1)
plot(t, X(4,:))
hold on
plot(time,vicon(7,:),'r')

subplot(3,1,2)
plot(t,X(5,:))
hold on
plot(time,vicon(8,:),'r')

subplot(3,1,3)
plot(t, X(6,:))
hold on
plot(time,vicon(9,:),'r')

figure,
subplot(3,1,1)
plot(t, X(7,:))
hold on
plot(time,vicon(4,:),'r')

subplot(3,1,2)
plot(t,X(8,:))
hold on
plot(time,vicon(5,:),'r')

subplot(3,1,3)
plot(t, X(9,:))
hold on
plot(time,vicon(6,:),'r')




