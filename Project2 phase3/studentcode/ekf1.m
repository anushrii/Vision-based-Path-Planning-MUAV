function [X, Z] = ekf1(sensor, vic, worldnew, func)
% EKF1 Extended Kalman Filter with Vicon velocity as inputs
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
%   vic    - struct for storing vicon linear velocity in world frame and
%            angular velocity in body frame, fields include
%          - t: vicon timestamp
%          - vel = [vx; vy; vz; wx; wy; wz]
%   varargin - any variables you wish to pass into the function, could be
%              a data structure to represent the map or camera parameters,
%              your decision. But for the purpose of testing, since we don't
%              know what inputs you will use, you have to specify them in
%              init_script by doing
%              ekf1_handle = ...
%                  @(sensor, vic) ekf1(sensor, vic, your input arguments);
%
% OUTPUTS:
% X - nx1 state of the quadrotor, n should be greater or equal to 6
%     the state should be in the following order
%     [x; y; z; roll; pitch; yaw; other states you use]
%     we will only take the first 6 rows of X
% OPTIONAL OUTPUTS:
% Z - mx1 measurement of your pose estimator, m shoulb be greater or equal to 6
%     the measurement should be in the following order
%     [x; y; z; roll; pitch; yaw; other measurement you use]
%     note that this output is optional, it's here in case you want to log your
%     measurement 

persistent cov_old mu_old told fold

if(isempty(mu_old))
  mu_old = zeros(6,1); 
  cov_old = 100*eye(6); 
  told = vic.t;
  fold = [0;0;0;0;0;0];
end
[pos, eul, ~] = estimate_pose(sensor, worldnew);
ZS = [pos;eul];
%% PROCESS MODEL & input
% Assumptions:  Vicon system gives a noisy estimate of the linear velocity
% gives a noisy estimate of the angular velocity.

% world frame linear velocity and body frame angular from the vicon data.
% vic.vel
% xdot = vic.vel;
omega = vic.vel(4:6,:);
phi = mu_old(4);
theta = mu_old(5);
psi = mu_old(6);

G = [cos(theta) 0 -cos(phi)*sin(theta);
    0  1 sin(phi);
    sin(theta) 0 cos(phi)*cos(theta)];
xdot =  inv(G)*omega;

% u = input_fun(Ts(i));
% % x = XS(i);
%  % xpred = F * x + G * u
%  % xdot  = A * x + B * (u + n) 
 %% prediction
% mu  = F*mu_old + G*u;\

% Vx = vic.vel(1,:);
% Vy = vic.vel(2,:);
% Vz = vic.vel(3,:);

% f = [vic.vel(1:3,:);xdot ];

wx = vic.vel(4,:);
wy = vic.vel(5,:);
wz = vic.vel(6,:);



dt = (vic.t- told );
mu = mu_old + (vic.t- told )*(fold);
fold = [vic.vel(1:3,:);xdot ];
% (vic.t- told )
%===================================================================================

Q    = 1000*eye(6);
R    = 1*eye(6);
Ct   = eye(6); 
%===================================================================================
F = eye(6) + func(phi,theta,wx,wz)*dt;  %  @(phi,theta,wx,wz)
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
 told = vic.t;

X =  mu_new';
Z = zeros(6,1);

end
