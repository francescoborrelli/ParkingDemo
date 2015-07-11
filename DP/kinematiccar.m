% Georg Schildbach, 14/Dec/2013 --- Automobile
% Simulates the kinematic car
% --------------------------------------------------------------------------------------------------
% dt: total simulation time
% x,y,phi: initial coordinates
% v: velocity [m/s]
% a: acceleration [m/s^2]
% delta: front wheel steering angle [rad]
% auto: structure including geometric data
% --------------------------------------------------------------------------------------------------
% xn, yn, phin, vn: new coordinates
% --------------------------------------------------------------------------------------------------

function [xn,yn,phin,vn] = kinematiccar(dt,x,y,phi,v,a,delta,auto)
   opts = odeset('RelTol',1e-2,'AbsTol',[1e-2 1e-2 1e-3 1e-2]);
   [T,XT] = ode45(@(t,X)diffmodel(t,X,a,delta,auto.d,auto.dmax),[0 dt],[x y phi v]');
   xn =   XT(size(XT,1),1);
   yn =   XT(size(XT,1),2);
   phin = mod(XT(size(XT,1),3),2*pi);
   vn =   XT(size(XT,1),4);
end

% ==================================================================================================

function Xd = diffmodel(t,X,a,delta,d,dmax)  %
   Xd = 0*X;
   if delta > dmax
      delta = dmax;
   elseif delta < -dmax
      delta = -dmax;
   end
   Xd(1) = cos(X(3))*X(4);      % dx/dt
   Xd(2) = sin(X(3))*X(4);      % dy/dt
   Xd(3) = tan(delta)/d*X(4);   % dphi/dt
   Xd(4) = a;                   % dv/dt
end