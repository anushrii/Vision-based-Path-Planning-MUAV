function [vel, omg, t, MSE,n] = estimate_vel(sensor, worldnew,R_c_w,T)
%ESTIMATE_VEL 6DOF velocity estimator
%   sensor - struct stored in provided dataset, fields include
%          - is_ready: logical, indicates whether sensor data is valid
%          - t: timestamp
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
%              estimate_vel_handle = ...
%                  @(sensor) estimate_vel(sensor, your personal input arguments);
%   vel - 3x1 velocity of the quadrotor in world frame
%   omg - 3x1 angular velocity of the quadrotor
persistent tracker oldsensor oldpoints corners 
K = [314.1779 0         199.4848; ...
0         314.2218  113.7838; ...
0         0         1];
ids = sensor.id;

if(size(ids,2)== 0)
    n =0;
    MSE = 0;
    vel = zeros(3,1);
    omg = zeros(3,1);
%     vel = [];
%     omg = [];
   t = sensor.t;
else
    
if isempty(corners)
    vel = zeros(3,1);
    omg = zeros(3,1);
    t = sensor.t;
    tracker = vision.PointTracker('MaxBidirectionalError', 1);
     
     
    %------------------------detecting corners for Initiallizing a tracker-------------------------------
%     points = detectMinEigenFeatures(sensor.img);
%     corners = points.selectStrongest(round(size(points,1)));
%     initialize(tracker, corners.Location, sensor.img);
%     oldpoints = corners.Location;
    
    %--------------------------------- Initiallizing a tracker  ---------------------------------------------------
   
    corners = corner(sensor.img, 'harris');
    initialize(tracker, corners, sensor.img);
    oldpoints = corners;
    
    
    oldsensor = sensor;
    MSE = 0;
    n = 0;
else
 %%%%%=====================================================================================%%%%%%%
%  pixel_xy =[];
% for i= 1:size(sensor.p0,2)
%     pixel  = [sensor.p0(:,i)' ; sensor.p4(:,i)'; sensor.p1(:,i)';...
%     sensor.p2(:,i)'; sensor.p3(:,i)'];
%     pixel_xy = [pixel_xy; pixel];
% end
% 
% % giving the sensor data as pixel x's and y's
% x = pixel_xy(:,1)';
% y = pixel_xy(:,2)';
% 
% %-------------------------------------------------------------------------------------------------
% 
% 
% world_xy = zeros(5*size(ids,2),2);
% for i = 0:size(ids,2)-1
%    index = ids(1,i + 1);
% world_xy((1 + (i)*5):(5 + (i)*5),:)=worldnew{index+1} ;
% %    plot(world_xy(:,1),world_xy(:,2) , '*');
% end
% 
% % giving the apriltag loactions as wolrd X's and Y's
% X = world_xy(:,1)';
% Y = world_xy(:,2)';
% 
% %----------------------------------------------------------------------------------------------
% % calculating homography between the world points 
% % and the image pixel co-ordinates by the SVD method.
% 
% n = size(X, 2);
% rows0 = zeros(3, n);
% rowsXY = -[X; Y; ones(1,n)];
% hx = [rowsXY; rows0; x.*X; x.*Y; x];
% hy = [rows0; rowsXY; y.*X; y.*Y; y];
% h = [hx hy];
% [U, ~, ~] = svd(h, 'econ');
% H = (reshape(U(:,9), 3, 3)).';
% %--------------------------------------------------------------------------------------------------------
% % Making an S03 rotation matrix from the homography obtained
% H = H/H(3,3);
% % K^-1*H = (r1, r2, T)
% % Hdash = (r1 r2 T);
% 
% Hdash = K\H;
% 
% 
% Rdash  = [Hdash(:,1), Hdash(:,2), cross(Hdash(:,1),Hdash(:,2))];
% [U , ~ , V] = svd(Rdash);
% %-------------------------------------------------------------------------------------------------------
% % Camera = R_c_w* World
% R_c_w = U*[1 0 0 ; 0 1 0; 0 0 det(U*V')]*V';
% T = Hdash(:,3)/(norm(Hdash(:,1)));
% H_c_w = [R_c_w , T; 0 0 0 1]; % world in camera * cam in  body
% 
% R_r_c = [ cosd(45), -sind(45), 0;
%         -sind(45), -cosd(45), 0;
%          0        , 0     ,  -1];
% T_r_c =  [ -0.04; 0 ; -0.03];
% H_r_c = [R_r_c , T_r_c; 0 0 0 1];
% 
% 
% % Hfinal1 = inv(H_r_c*H_c_w);                                                                                                      

%%%%%=================================================================================================%%%%%%%

%------------------------------------------- Tracking corners -----------------------------------------------
    frame = sensor.img;
    [pointsTracked, validity] = step(tracker, frame);
    
     if (numel(find(validity))< 80)
%------------------------------------- Re-Initiallizing a tracker  -----------------------------------------   
     release(tracker);
     tracker = vision.PointTracker('MaxBidirectionalError', 1);
%-------------------------------------- detecting new corners ----------------------------------------------     
%        points = detectMinEigenFeatures(oldsensor.img); 
%        corners = points.selectStrongest(round(size(points,1)));
%        initialize(tracker, corners.Location, oldsensor.img);
%        oldpoints = corners.Location;
       
     corners = corner(sensor.img, 'harris');
     initialize(tracker, corners, sensor.img);
     initialize(tracker, corners, oldsensor.img);
     oldpoints = corners;
      
     [pointsTracked, validity] = step(tracker, frame);
    end

    %-----------------------------checking for inliers using ransac-----------------------------------------
    % RANSAC is the default inlier detector in this 
    % H = vision.GeometricTransformEstimator;
    pointsTracked = pointsTracked((validity(:,1)),:);
    oldpoints = oldpoints((validity(:,1)),:);

    %---------------------------setting tracked valid points for the tracker-----------------------------
    setPoints(tracker,pointsTracked) % ,inlierIndx);
%     size(find(inlierIndx))
%     pointsTracked =  pointsTracked(inlierIndx,:);
%     oldpoints = oldpoints(inlierIndx,:);

%     out = insertMarker(frame,pointsTracked, '+');
%     drawnow
%     imshow(out);
    
    ptsTrackCamera =  (K)\([pointsTracked';ones(1,size(pointsTracked,1)) ]);
    num = R_c_w'*T;
    den = R_c_w(:,3)'*ptsTrackCamera;
    ptsTracKZ  =   bsxfun(@rdivide, num(end),den);

    xT = ptsTrackCamera(1,:);
    yT = ptsTrackCamera(2,:);
    zT = ptsTracKZ;
    
    f_1 = [(-bsxfun(@rdivide, 1,zT ))', zeros(size(zT,2),1), (bsxfun(@rdivide, xT,zT ))'...
            (bsxfun(@times, xT,yT ))', (-(ones(1,size(xT,2)) + bsxfun(@times, xT,xT )))', yT' ];
    f_2 = [zeros(size(zT,2),1),  (-bsxfun(@rdivide, 1,zT ))',  (bsxfun(@rdivide, yT,zT ))'...
            (ones(1,size(yT,2)) + bsxfun(@times, yT,yT ))' ,  (-bsxfun(@times, xT,yT ))' , -xT'];
    f = [f_1;f_2]; 

    
    oldPtsCamera =  (K)\([oldpoints';ones(1,size(oldpoints,1)) ]);
   
    Vx = (ptsTrackCamera(1,:) - oldPtsCamera(1,:))/0.02; % try with 0.02
    Vy = (ptsTrackCamera(2,:) - oldPtsCamera(2,:))/0.02;
   
    
    V = [Vx'; Vy']; 
    velocities = f\V;
    
    %--------------------------------3-point RANSAC------------------------------------------------
    [inliers, MSE,n] = threePtRansac (xT,yT,zT,Vx,Vy, velocities);
    
    if (~isempty(inliers))
    inliers = reshape(inliers,[],1);

    xI =  xT(inliers);
    yI =  yT(inliers);
    zI =  zT(inliers);
    % recomputing the velocities
    
    f_1_R = [(-bsxfun(@rdivide, 1,zI ))', zeros(size(zI,2),1), (bsxfun(@rdivide, xI,zI ))'...
            (bsxfun(@times, xI,yI ))', (-(ones(1,size(xI,2)) + bsxfun(@times, xI,xI )))', yI' ];
    f_2_R = [zeros(size(zI,2),1),  (-bsxfun(@rdivide, 1,zI ))',  (bsxfun(@rdivide, yI,zI ))'...
            (ones(1,size(yI,2)) + bsxfun(@times, yI,yI ))' ,  (-bsxfun(@times, xI,yI ))' , -xI'];
    f_R = [f_1_R;f_2_R]; 
    
    
    Vx_R = Vx(inliers);
    Vy_R = Vy(inliers);
    
    V_R = [Vx_R';Vy_R'];
    velocities = f_R\V_R;
    
    vel =  (R_c_w)\ velocities(1:3);
    omg =  (R_c_w)\ velocities(4:6);
     t = sensor.t;
    oldsensor = sensor;
    oldpoints = pointsTracked; 
     
    else
    vel = zeros(3,1);
    omg = zeros(3,1);   

    t = sensor.t;

    oldsensor = sensor;
    oldpoints = pointsTracked;

    end
end
  
   
end
