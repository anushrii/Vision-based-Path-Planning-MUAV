%% 3 point RANSAC for velocity estimation

function [inliers,MSE,n] = threePtRansac (x,y,z,xdot,ydot, velocities)
n = 50; % number of iterations
error = 0.001;
 inliers = logical(zeros(size(x,2),1));
for i=1:n
    
    if size(x,2) >3
randomPtIdx = randperm(size(x,2),3);
xR = x(randomPtIdx);
yR = y(randomPtIdx);
zR = z(randomPtIdx);
% xyzR = [xR;  yR ; zR];
xdotR = xdot(randomPtIdx);
ydotR = ydot(randomPtIdx);
% VR = [xdotR;ydotR];

f_1 = [(-bsxfun(@rdivide, 1,zR ))', zeros(size(zR,2),1), (bsxfun(@rdivide, xR,zR ))'...
            (bsxfun(@times, xR,yR ))', (-(ones(1,size(xR,2)) + bsxfun(@times, xR,xR )))', yR' ];
f_2 = [zeros(size(zR,2),1),  (-bsxfun(@rdivide, 1,zR ))',  (bsxfun(@rdivide, yR,zR ))'...
            (ones(1,size(yR,2)) + bsxfun(@times, yR,yR ))' ,  (-bsxfun(@times, xR,yR ))' , -xR'];
f = [f_1;f_2]; 

Vr = f*velocities;

MSE = mean(mean(([xdotR';ydotR'] - Vr).^2,2),1);
% disp(MSE)
    if MSE < error
        
         inliers(randomPtIdx) = 1; 
    end

    else
        inliers = [];
        MSE =0;
        n= 0;
        
    end
end