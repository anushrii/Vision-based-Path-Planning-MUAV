function [filteredOutput] = mykalman1(XS,ZS,Ts, n)

% XS -> state of the system
% ZS -> measurement model -> x,xdot, xddot, xdddot.


initialize;
mu_old = [ 0, 0, 0,0]';
cov_old = P0;
% Kt = eye(4);
Ct = [1;0;0;0]';
% Kt =  [1;0;0;0];
% Kt = eye(4);
% cov_old = [1;0;0;0];
%
%  F=F';


initialize;
if  n==1
    mu_old = XS(:,1); 
    cov_old = P0;
    Ct = eye(4);
elseif n==2
    mu_old = [ 0, 0, 0,0]';
    cov_old = P0;
    Ct = [1;0;0;0]';  
    R = 1000;
elseif n==3
    mu_old = XS(1:4,1);
    cov_old = P0;
    Ct = [0,0,0;
          1,0,0 ;
          0,1,0;
          0, 0, 1]';  
    R = 1000* eye(3);
    
end

    
for i=1:size(Ts,2)
    

%% prediction
u = input_fun(Ts(i));

mu  = F*mu_old + G*u;
cov = F*cov_old*F' +  Q.*(dt^2);  %V*Q*V'  === here V = dt; use Qdt


%% update
Kt = cov*Ct'*(Ct*cov*Ct' + R)^-1;
if n==1
mu_new = mu + Kt*(ZS(:,i) - Ct*mu)   ;
elseif n==2
 mu_new = mu + Kt*(ZS(1,i) - Ct*mu)   ;   
elseif n==3
  mu_new = mu + Kt*(ZS(2:4,i) - Ct*mu)   ;  
end
cov_new  = cov - Kt*Ct*cov;

 filteredOutput(i,:) = mu_new;
 mu_old = mu_new;
 cov_old = cov_new;
end

end