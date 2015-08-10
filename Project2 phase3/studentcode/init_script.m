% Add additional inputs after the given ones if you want to
% Example:
% your_input = 1;
% ekf_handle1 = @(sensor, vic) ekf1(sensor, vic, your_input);
% ekf_handle2 = @(sensor) ekf2(sensor, your_input);
%
% We will only call ekf_handle in the test function.
% Note that this will only create a function handle, but not run the function


% vic ?
% vic = struct(field,value)

a = 0.152;
b = 0.178;
 world  = cell(24,18);
for j=1:5
    for i = 1:24
        
        world{i,j} = [ (((i-1)*a + i*a)/2), (((j-1)*a +j*a )/2) ; 
            (i-1)*a,(j-1)*a; 
            i*a, (j-1)*a;
            i*a, j*a ;
            (i-1)*a,j*a];
    end 
 end

for j=6
    for i = 1:24
        
        world{i,j} = [ (((i-1)*a + i*a)/2), (((j-1)*a + ((j-1)*a +b) )/2) ;
            (i-1)*a,(j-1)*a; 
            i*a, (j-1)*a;
            i*a, (j-1)*a + b ;
            (i-1)*a,(j-1)*a + b];
    end 
end


for j=7:11
    for i = 1:24
        
       world{i,j} = [ (((i-1)*a + i*a)/2), (((j-2)*a +b +(j-1)*a +b)/2) ;
           (i-1)*a,(j-2)*a + b; 
            i*a, (j-2)*a + b;
            i*a, (j-1)*a + b ;
            (i-1)*a,(j-1)*a + b];
    end 
end

for j=12
    for i = 1:24
        
        world{i,j} = [ (((i-1)*a + i*a)/2), (((j-2)*a +(j-2)*a +3*b)/2) ;
            (i-1)*a,((j-2)*a +b); 
            i*a, ((j-2)*a +b );
            i*a, ((j-2)*a + 2*b) ;
            (i-1)*a,((j-2)*a + 2*b)];
   end
end
            
  

for j=13:18
    for i = 1:24
        
       world{i,j} = [ (((i-1)*a + i*a)/2), (((j-3)*a +(j-2)*a + 4*b)/2) ;
       (i-1)*a,(j-3)*a + 2*b; 
            i*a, (j-3)*a + 2*b;
            i*a, (j-2)*a + 2*b;
            (i-1)*a,(j-2)*a + 2*b];
    end 
end

 worldnew = world (1:2:24, 1:2: 18);
figure,
 axis([0 4 0 4])
 for i = 1:108
     
plot(worldnew{i}(1,1)',worldnew{i}(1,2)','o')
hold on
drawnow;
% pause(0.25)
 end
estimate_pose_handle = @(sensor) estimate_pose(sensor,worldnew);


%=========================================================================%
syms Vx Vy Vz wx wy wz phi theta psi x y z omg1 omg2 omg3 acc1 acc2 acc3

G = [cos(theta) 0 -cos(phi)*sin(theta);
    0  1 sin(phi);
    sin(theta) 0 cos(phi)*cos(theta)];

ang = [wx ; wy; wz];
processModel = [Vx ; Vy ; Vz  ; inv(G)*ang];
state = [ x ; y; z; phi ; theta;  psi];
A = simplify(jacobian(processModel , state));
func = matlabFunction(A); %  @(phi,theta,wx,wz)

%=========================================================================%
omg = [omg1 ; omg2; omg3];
acc = [acc1 ; acc2; acc3 ];

% accel = acc + [0;0;-9.8];


R = RPYtoRot_ZXY(phi,theta,psi);
% accel = R'*accel ;
accel = R.'*acc  + [0;0;-9.8];
processModel = [ Vx;Vy;Vz;inv(G)*omg; accel];
state = [  x ; y; z ; phi ; theta;  psi; Vx; Vy; Vz];
A = simplify(jacobian(processModel , state));
func2 = matlabFunction(A); %   @(acc1,acc2,acc3,omg1,omg3,phi,psi,theta)





ekf1_handle = @(sensor, vic) ekf1(sensor,vic ,worldnew, func);
ekf2_handle = @(sensor) ekf2(sensor ,worldnew, func2);



% [ 0, 0, 0,                                                                                                         0,                                                                                                                                                           0,                                                                                                                     0, 1, 0, 0]
% [ 0, 0, 0,                                                                                                         0,                                                                                                                                                           0,                                                                                                                     0, 0, 1, 0]
% [ 0, 0, 0,                                                                                                         0,                                                                                                                                                           0,                                                                                                                     0, 0, 0, 1]
% [ 0, 0, 0,                                                                                                         0,                                                                                                                           omg3*cos(theta) - omg1*sin(theta),                                                                                                                     0, 0, 0, 0]
% [ 0, 0, 0,                                                           -(omg3*cos(theta) - omg1*sin(theta))/cos(phi)^2,                                                                                                     (sin(phi)*(omg1*cos(theta) + omg3*sin(theta)))/cos(phi),                                                                                                                     0, 0, 0, 0]
% [ 0, 0, 0,                                                 (sin(phi)*(omg3*cos(theta) - omg1*sin(theta)))/cos(phi)^2,                                                                                                               -(omg1*cos(theta) + omg3*sin(theta))/cos(phi),                                                                                                                     0, 0, 0, 0]
% [ 0, 0, 0, sin(phi)*sin(theta)*(acc3 - 49/5) + acc2*cos(phi)*cos(psi)*sin(theta) - acc1*cos(phi)*sin(psi)*sin(theta), - acc1*(cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi)) - acc2*(sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi)) - cos(phi)*cos(theta)*(acc3 - 49/5), acc2*(cos(psi)*cos(theta) - sin(phi)*sin(psi)*sin(theta)) - acc1*(cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta)), 0, 0, 0]
% [ 0, 0, 0,                                  cos(phi)*(acc3 - 49/5) - acc2*cos(psi)*sin(phi) + acc1*sin(phi)*sin(psi),                                                                                                                                                           0,                                                                             -cos(phi)*(acc1*cos(psi) + acc2*sin(psi)), 0, 0, 0]
% [ 0, 0, 0, acc1*cos(phi)*cos(theta)*sin(psi) - acc2*cos(phi)*cos(psi)*cos(theta) - cos(theta)*sin(phi)*(acc3 - 49/5),   acc1*(cos(psi)*cos(theta) - sin(phi)*sin(psi)*sin(theta)) + acc2*(cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta)) - cos(phi)*sin(theta)*(acc3 - 49/5), acc2*(cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi)) - acc1*(sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi)), 0, 0, 0]


% accel = R*acc  + [0;0;-9.8];
% [ 0, 0, 0,                                                                             0,                                                                                                                                                  0,                                                                                                                     0, 1, 0, 0]
% [ 0, 0, 0,                                                                             0,                                                                                                                                                  0,                                                                                                                     0, 0, 1, 0]
% [ 0, 0, 0,                                                                             0,                                                                                                                                                  0,                                                                                                                     0, 0, 0, 1]
% [ 0, 0, 0,                                                                             0,                                                                                                                  omg3*cos(theta) - omg1*sin(theta),                                                                                                                     0, 0, 0, 0]
% [ 0, 0, 0,                               -(omg3*cos(theta) - omg1*sin(theta))/cos(phi)^2,                                                                                            (sin(phi)*(omg1*cos(theta) + omg3*sin(theta)))/cos(phi),                                                                                                                     0, 0, 0, 0]
% [ 0, 0, 0,                     (sin(phi)*(omg3*cos(theta) - omg1*sin(theta)))/cos(phi)^2,                                                                                                      -(omg1*cos(theta) + omg3*sin(theta))/cos(phi),                                                                                                                     0, 0, 0, 0]
% [ 0, 0, 0,  sin(theta)*(acc3*sin(phi) + acc2*cos(phi)*cos(psi) - acc1*cos(phi)*sin(psi)), - acc1*(cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi)) - acc2*(sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi)) - acc3*cos(phi)*cos(theta), acc2*(cos(psi)*cos(theta) - sin(phi)*sin(psi)*sin(theta)) - acc1*(cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta)), 0, 0, 0]
% [ 0, 0, 0,               acc3*cos(phi) - acc2*cos(psi)*sin(phi) + acc1*sin(phi)*sin(psi),                                                                                                                                                  0,                                                                             -cos(phi)*(acc1*cos(psi) + acc2*sin(psi)), 0, 0, 0]
% [ 0, 0, 0, -cos(theta)*(acc3*sin(phi) + acc2*cos(phi)*cos(psi) - acc1*cos(phi)*sin(psi)),   acc1*(cos(psi)*cos(theta) - sin(phi)*sin(psi)*sin(theta)) + acc2*(cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta)) - acc3*cos(phi)*sin(theta), acc2*(cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi)) - acc1*(sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi)), 0, 0, 0]


%===========================================
% [ 0, 0, 0,                                                                             0,                                                                                                                                                  0,                                                                                                                     0, 1, 0, 0]
% [ 0, 0, 0,                                                                             0,                                                                                                                                                  0,                                                                                                                     0, 0, 1, 0]
% [ 0, 0, 0,                                                                             0,                                                                                                                                                  0,                                                                                                                     0, 0, 0, 1]
% [ 0, 0, 0,                                                                             0,                                                                                                                  omg3*cos(theta) - omg1*sin(theta),                                                                                                                     0, 0, 0, 0]
% [ 0, 0, 0,                               -(omg3*cos(theta) - omg1*sin(theta))/cos(phi)^2,                                                                                            (sin(phi)*(omg1*cos(theta) + omg3*sin(theta)))/cos(phi),                                                                                                                     0, 0, 0, 0]
% [ 0, 0, 0,                     (sin(phi)*(omg3*cos(theta) - omg1*sin(theta)))/cos(phi)^2,                                                                                                      -(omg1*cos(theta) + omg3*sin(theta))/cos(phi),                                                                                                                     0, 0, 0, 0]
% [ 0, 0, 0,  sin(theta)*(acc3*sin(phi) + acc2*cos(phi)*cos(psi) - acc1*cos(phi)*sin(psi)), - acc1*(cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi)) - acc2*(sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi)) - acc3*cos(phi)*cos(theta), acc2*(cos(psi)*cos(theta) - sin(phi)*sin(psi)*sin(theta)) - acc1*(cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta)), 0, 0, 0]
% [ 0, 0, 0,               acc3*cos(phi) - acc2*cos(psi)*sin(phi) + acc1*sin(phi)*sin(psi),                                                                                                                                                  0,                                                                             -cos(phi)*(acc1*cos(psi) + acc2*sin(psi)), 0, 0, 0]
% [ 0, 0, 0, -cos(theta)*(acc3*sin(phi) + acc2*cos(phi)*cos(psi) - acc1*cos(phi)*sin(psi)),   acc1*(cos(psi)*cos(theta) - sin(phi)*sin(psi)*sin(theta)) + acc2*(cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta)) - acc3*cos(phi)*sin(theta), acc2*(cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi)) - acc1*(sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi)), 0, 0, 0]
 
%================================
% [                                                                             0,                                                                                                                                                  0,                                                                                                                     0, 0, 0, 0, 1, 0, 0]
% [                                                                             0,                                                                                                                                                  0,                                                                                                                     0, 0, 0, 0, 0, 1, 0]
% [                                                                             0,                                                                                                                                                  0,                                                                                                                     0, 0, 0, 0, 0, 0, 1]
% [                                                                             0,                                                                                                                  omg3*cos(theta) - omg1*sin(theta),                                                                                                                     0, 0, 0, 0, 0, 0, 0]
% [                               -(omg3*cos(theta) - omg1*sin(theta))/cos(phi)^2,                                                                                            (sin(phi)*(omg1*cos(theta) + omg3*sin(theta)))/cos(phi),                                                                                                                     0, 0, 0, 0, 0, 0, 0]
% [                     (sin(phi)*(omg3*cos(theta) - omg1*sin(theta)))/cos(phi)^2,                                                                                                      -(omg1*cos(theta) + omg3*sin(theta))/cos(phi),                                                                                                                     0, 0, 0, 0, 0, 0, 0]
% [  sin(theta)*(acc3*sin(phi) + acc2*cos(phi)*cos(psi) - acc1*cos(phi)*sin(psi)), - acc1*(cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi)) - acc2*(sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi)) - acc3*cos(phi)*cos(theta), acc2*(cos(psi)*cos(theta) - sin(phi)*sin(psi)*sin(theta)) - acc1*(cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta)), 0, 0, 0, 0, 0, 0]
% [               acc3*cos(phi) - acc2*cos(psi)*sin(phi) + acc1*sin(phi)*sin(psi),                                                                                                                                                  0,                                                                             -cos(phi)*(acc1*cos(psi) + acc2*sin(psi)), 0, 0, 0, 0, 0, 0]
% [ -cos(theta)*(acc3*sin(phi) + acc2*cos(phi)*cos(psi) - acc1*cos(phi)*sin(psi)),   acc1*(cos(psi)*cos(theta) - sin(phi)*sin(psi)*sin(theta)) + acc2*(cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta)) - acc3*cos(phi)*sin(theta), acc2*(cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi)) - acc1*(sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi)), 0, 0, 0, 0, 0, 0]
