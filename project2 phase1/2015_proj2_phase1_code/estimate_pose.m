function [pos, eul, t, R_c_w,T] = estimate_pose(sensor, worldnew)
%ESTIMATE_POSE 6DOF pose estimator based on apriltags
%   sensor - struct stored in provided dataset, fields include
%          - is_ready: logical, indicates whether sensor data is valid
%          - rpy, omg, acc: imu readings, you should not use these in this phase
%          - img: uint8, 240x376 grayscale image
%          - id: 1xn ids of detected tags
%          - p0, p1, p2, p3, p4: 2xn pixel position of center and
%                                four corners of detected tags
%            Y
%            ^ P3 == P2
%            | || P0 ||
%            | P4 == P1
%            o---------> X
%   varargin - any variables you wish to pass into the function, could be
%              a data structure to represent the map or camera parameters,
%              your decision. But for the purpose of testing, since we don't
%              know what inputs you will use, you have to specify them in
%              init_script by doing
%              estimate_pose_handle = ...
%                  @(sensor) estimate_pose(sensor, your personal input arguments);
%   pos - 3x1 position of the quadrotor in world frame
%   eul - 3x1 euler angles of the quadrotor
% H = est_homography(X,Y,x,y)


% camera parameters
K = [314.1779 0         199.4848; ...
0         314.2218  113.7838; ...
0         0         1];
ids = sensor.id;
if(size(ids)~= 0)
 
%----------------------------------------------------------
pixel_xy =[];
for i= 1:size(sensor.p0,2)
    pixel  = [sensor.p0(:,i)' ; sensor.p4(:,i)'; sensor.p1(:,i)';...
        sensor.p2(:,i)'; sensor.p3(:,i)'];
    pixel_xy = [pixel_xy; pixel];
end



% giving the sensor data as pixel x's and y's
x = pixel_xy(:,1)';
y = pixel_xy(:,2)';

%----------------------------------------------------------

% figure,
world_xy = zeros(5*size(ids,2),2);
for i = 0:size(ids,2)-1
   index = ids(1,i + 1);
world_xy((1 + (i)*5):(5 + (i)*5),:)=worldnew{index+1} ;
%    plot(world_xy(:,1),world_xy(:,2) , '*');
end

% giving the apriltag loactions as wolrd X's and Y's
X = world_xy(:,1)';
Y = world_xy(:,2)';


 
% plot(x,y, 'o');
% % figure,
% % plot(X,Y, 'x');
% drawnow;
% % axis([0 500 0 300])
% axis([0 3.5 0 3])

%----------------------------------------------------------
% calculating homography between the world points 
% and the image pixel co-ordinates by the SVD method.
n = size(X, 2);
rows0 = zeros(3, n);
rowsXY = -[X; Y; ones(1,n)];
hx = [rowsXY; rows0; x.*X; x.*Y; x];
hy = [rows0; rowsXY; y.*X; y.*Y; y];
h = [hx hy];
[U, ~, ~] = svd(h, 'econ');
H = (reshape(U(:,9), 3, 3)).';
%----------------------------------------------------------
% Making an S03 rotation matrix from the homography obtained
H = H/H(3,3);
% K^-1*H = (r1, r2, T)
% Hdash = (r1 r2 T);

Hdash = K\H;


Rdash  = [Hdash(:,1), Hdash(:,2), cross(Hdash(:,1),Hdash(:,2))];
[U , ~ , V] = svd(Rdash);
%----------------------------------------------------------
% Camera = R_c_w* World
R_c_w = U*[1 0 0 ; 0 1 0; 0 0 det(U*V')]*V';
T = Hdash(:,3)/(norm(Hdash(:,1)));
H_c_w = [R_c_w , T; 0 0 0 1]; % world in camera * cam in  body

% display(Hdash(:,3)'*cross(Hdash(:,1),Hdash(:,2)))
% display(det(Hdash));
%----------------------------------------------------------
% we now have, Camera = R* World
% Robot = R_r_c * Camera;
R_r_c = [ cosd(45), -sind(45), 0;
        -sind(45), -cosd(45), 0;
         0        , 0     ,  -1];
T_r_c =  [ 0.04; 0 ; -0.03];
H_r_c = [R_r_c , T_r_c; 0 0 0 1];


Hfinal1 = inv(H_r_c*H_c_w);
 
%----------------------------------------------------------  
% for sanity check of the transformations
% Tform = maketform('projective',H');
% J = imtransform(sensor.img,Tform);
% % % T = projective2d(Rfinal);
% % 
% % % J = imwarp(sensor.img,T);
% % 
% imshow(J);

%---------------------------------------------------------
% final outputs of the function
pos = Hfinal1(1:3,end);
[phi,theta,psi] = RotToRPY_ZXY(Hfinal1(1:3,1:3)');
eul = [phi,theta,psi]'; 
t = sensor.t;


else
%----------------------------------------------------------
%if the apriltages have null information
pos = [];

eul = []; 
t = sensor.t;
    
    R_c_w = [];
    T =[];
    
end
