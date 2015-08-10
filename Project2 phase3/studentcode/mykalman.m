function [filteredOutput] = mykalman(XS,ZS,Ts)

% XS -> state of the system
% ZS -> measurement model -> x,xdot, xddot, xdddot.


initialize;
mu_old = XS(:,1);   %[ 0, 0, 0,0]';
% cov_old =[0, 0,0,0]';
cov_old = P0;
% if num==1
Ct = eye(4);
% end
Kt = eye(4);
% if num==2
% Ct = [1;0;0;0];
% end

for i=1:size(Ts,2)
    
    
u = input_fun(Ts(i));
% x = XS(i);
 % xpred = F * x + G * u
 % xdot  = A * x + B * (u + n) 
%% prediction
mu  = F*mu_old + G*u;
cov = F*cov_old*F' +  Q.*(dt^2);  %V*Q*V'  === here V = dt; use Qdt





%% update
 Kt = cov*Ct'*(Ct*cov*Ct' + R)^-1;
 mu_new = mu + Kt*(ZS(:,i) - Ct*mu)   ;
 cov_new  = cov - Kt*Ct*cov;

 filteredOutput(i,:) = mu_new;   
 mu_old = mu_new;
 cov_old = cov_new;
end


end