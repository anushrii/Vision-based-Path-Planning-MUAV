function [X, Z] = ekf2(sensor, worldnew,func)
% EKF2 Extended Kalman Filter with IMU as inputs
%
% INPUTS:
%   sensor - struct stored in provided dataset, fields include
%          - is_ready: logical, indicates whether sensor data is valid
%          - t: sensor timestamp
%          - rpy, omg, acc: imu readings
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
%              ekf1_handle = ...
%                  @(sensor) ekf2(sensor, your input arguments);
%
% OUTPUTS:
% X - nx1 state of the quadrotor, n should be greater or equal to 9
%     the state should be in the following order
%     [x; y; z; vx; vy; vz; roll; pitch; yaw; other states you use]
%     we will only take the first 9 rows of X
% OPTIONAL OUTPUTS:
% Z - mx1 measurement of your pose estimator, m shoulb be greater or equal to 6
%     the measurement should be in the following order
%     [x; y; z; roll; pitch; yaw; other measurement you use]
%     note that this output is optional, it's here in case you want to log your
%     measurement 
persistent cov_old mu_old told

if(isempty(mu_old))
  mu_old = zeros(9,1); 
  cov_old = 1*eye(9); 
  told = sensor.t;
end
[pos, eul, ~, R_c_w,T] = estimate_pose(sensor, worldnew);
dt = (sensor.t- told );
[vel, ~, ~, ~,~] = estimate_vel(sensor, worldnew,R_c_w,T);


% vel = (mu_old(1:3)- pos)*dt;
% vel = [0;0;0];
ZS = [pos;eul;vel];
%% PROCESS MODEL & input
% Assumptions:  Vicon system gives a noisy estimate of the linear velocity
% gives a noisy estimate of the angular velocity.

% world frame linear velocity and body frame angular from the vicon data.

omega = sensor.omg ;
phi = mu_old(4);
theta = mu_old(5);
psi = mu_old(6);

G = [cos(theta) 0 -cos(phi)*sin(theta);
    0  1 sin(phi);
    sin(theta) 0 cos(phi)*cos(theta)];
xdot =  inv(G)*omega;
vel_old = [mu_old(7);mu_old(8); mu_old(9)];
% u = input_fun(Ts(i));
% % x = XS(i);
%  % xpred = F * x + G * u
%  % xdot  = A * x + B * (u + n) 
 %% prediction
% mu  = F*mu_old + G*u;\

% Vx = vic.vel(1,:);
% Vy = vic.vel(2,:);
% Vz = vic.vel(3,:);
R_ = RPYtoRot_ZXY(phi,theta,psi);
% accel = R_*sensor.acc;
% accel = accel + [0;0;-9.8];

accel = sensor.acc;
accel = (R_'*(accel)) + [0;0;-9.8];
f = [ vel_old;xdot; accel];
% f = [xdot ;  ];

mu = mu_old + (sensor.t- told )*(f);
% (vic.t- told )
%===================================================================================
acc1 = sensor.acc(1);
acc2 = sensor.acc(2);
acc3 = sensor.acc(3);

omg1 = sensor.omg(1);
omg2 = sensor.omg(2);
omg3 = sensor.omg(3);

Q    = 1000*eye(9);
R    = 1*eye(9);
Ct   = eye(9); 
%===================================================================================
F = eye(9) + func(acc1,acc2,acc3,omg1,omg3,phi,psi,theta)*dt;  %  @(acc1,acc2,acc3,omg1,omg3,phi,psi,theta)
cov = F*cov_old*F' +  Q.*(dt^2);  %V*Q*V'  === here V = dt; use Qdt

%% update

%% PROCESS MEASUREMENT - Comes from the pose estimator using april tags.
if ~isempty(sensor) && (~sensor.is_ready || isempty(sensor.id))
   X =  mu_old';
   Z = zeros(6,1);
    return;
end

 Kt = cov*Ct'*(Ct*cov*Ct' + R)^-1;
 mu_new = mu + Kt*(ZS - Ct*mu)   ;
 cov_new  = cov - Kt*Ct*cov;

%  filteredOutput(i,:) = mu_new;   
 mu_old = mu_new;
 cov_old = cov_new;
 told = sensor.t;

X =  [mu_new(1:3);mu_new(7:9);mu_new(4:6) ]';
Z = zeros(6,1);


end
