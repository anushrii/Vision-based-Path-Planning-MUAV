%% 3 point RANSAC for velocity estimation

function [inliers,MSE,n] = threePtRansac (x,y,z,xdot,ydot, velocities)
n = 50; % number of iterations
error = 0.01;
 inliers = logical(zeros(size(x,2),1));
for i=1:n
    
    if size(x,2) >3
V = [xdot'; ydot'];
randomPtIdx = randperm(size(x,2),3);
xR = x(randomPtIdx);
yR = y(randomPtIdx);
zR = z(randomPtIdx);
% xyzR = [xR;  yR ; zR];
xdotR = xdot(randomPtIdx);
ydotR = ydot(randomPtIdx);
VR = [xdotR';ydotR'];

f_1 = [(-bsxfun(@rdivide, 1,zR ))', zeros(size(zR,2),1), (bsxfun(@rdivide, xR,zR ))'...
            (bsxfun(@times, xR,yR ))', (-(ones(1,size(xR,2)) + bsxfun(@times, xR,xR )))', yR' ];
f_2 = [zeros(size(zR,2),1),  (-bsxfun(@rdivide, 1,zR ))',  (bsxfun(@rdivide, yR,zR ))'...
            (ones(1,size(yR,2)) + bsxfun(@times, yR,yR ))' ,  (-bsxfun(@times, xR,yR ))' , -xR'];
f = [f_1;f_2]; 

vel = f\VR;

f_1a = [(-bsxfun(@rdivide, 1,z ))', zeros(size(z,2),1), (bsxfun(@rdivide, x,z ))'...
            (bsxfun(@times, x,y ))', (-(ones(1,size(x,2)) + bsxfun(@times, x,x )))', y' ];
f_2a = [zeros(size(z,2),1),  (-bsxfun(@rdivide, 1,z ))',  (bsxfun(@rdivide, y,z ))'...
            (ones(1,size(y,2)) + bsxfun(@times, y,y ))' ,  (-bsxfun(@times, x,y ))' , -x'];
fa = [f_1a;f_2a];

Vr = fa*vel;

MSE = mean(mean((V - Vr).^2,2),1);
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