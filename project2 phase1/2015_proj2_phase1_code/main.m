%% main
addpath('./data')
load('studentdata4.mat');

init_script
pos = zeros(3,size(data,2));
eul = zeros(3,size(data,2));
t = zeros(1,size(data,2));
for i=1:size(data,2)
    
   
[pos(:,i), eul(:,i) ,t(i)] = estimate_pose(data(1,i),worldnew);


end
% for j=1:40
%     imshow(data(1,j).img)
%      M(j) = getframe;
% end
% movie(M)
figure,
subplot(3,1,1)
plot(t, pos(1,:))
hold on
plot(time,vicon(1,:))

subplot(3,1,2)
plot(t, pos(2,:))
hold on
plot(time,vicon(2,:))

subplot(3,1,3)
plot(t, pos(3,:))
hold on
plot(time,vicon(3,:))

figure,
subplot(3,1,1)
plot(t, eul(1,:))
hold on
plot(time,vicon(4,:))

subplot(3,1,2)
plot(t, eul(2,:))
hold on
plot(time,vicon(5,:))

subplot(3,1,3)
plot(t, eul(3,:))
hold on
plot(time,vicon(6,:))




