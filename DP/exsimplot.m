% Georg Schildbach, 19/May/2015 --- DP for Parking
% Runs the simulation in Simulink and plots the results
% --------------------------------------------------------------------------------------------------
% REQUIRES 
% exdynprog.m
% --------------------------------------------------------------------------------------------------
% OUTPUTS
% simulation plots and animation
% --------------------------------------------------------------------------------------------------

clc
clear all
close all

% 1) Inputs and Options ----------------------------------------------------------------------------

% 1.1) Simulation

expathplan
SimOut = sim('simmodel','StartTime','0','StopTime','50')
X = SimOut.get('X');
U = SimOut.get('U');
C = SimOut.get('C');
R = SimOut.get('R');
T = SimOut.get('T');

% 1.2) Plot Options

% Trajectory
plottraj.linestyle = '-';
plottraj.linewidth = 1;
plottraj.color = [0 0 1]; % RGB value
plottraj.marker = 'none';

% Commands
plotcom.linestyle = '-';
plotcom.linewidth = 1;
plotcom.color = [0 0 0]; % RGB value
plotcom.marker = 'none';

% Reference Points
plotref.linestyle = '-';
plotref.linewidth = 1;
plotref.color = [1 0 0]; % RGB value
plotref.marker = '.';
plotref.markersize = 8;

% Reference Path
plotpath.linestyle = '-';
plotpath.linewidth = 1;
plotpath.color = [0 1 0]; % RGB value
plotpath.marker = 'none';

% Bounds
plotbound.linestyle = '-';
plotbound.linewidth = 3;
plotbound.color = [0 0 0]; % RGB value
plotbound.marker = 'none';

% 2) Output and Plotting ---------------------------------------------------------------------------------

% 2.1) Variable Definitions

N = max(size(T)');

% 2.2) Path Plot

fig = figure()
hold on
axis equal
xlabel('x [m]')
ylabel('y [m]')
title('Path Following (green: reference, red: reference points, blue: trajectory)')

% Obstacles
if (xyrange(1,2)-xyrange(1,1))>=(xyrange(1,4)-xyrange(1,3))
    x1 = xyrange(1,1)-1;
    x2 = xyrange(1,2)+1;
    y1 = xyrange(1,3)-(xyrange(1,2)-xyrange(1,1)-xyrange(1,4)+xyrange(1,3)+2)/2;
    y2 = xyrange(1,4)+(xyrange(1,2)-xyrange(1,1)-xyrange(1,4)+xyrange(1,3)+2)/2;
else
    x1 = xyrange(1,1)-(xyrange(1,4)-xyrange(1,3)-xyrange(1,2)+xyrange(1,1)+2)/2;
    x2 = xyrange(1,2)+(xyrange(1,4)-xyrange(1,3)-xyrange(1,2)+xyrange(1,1)+2)/2;
    y1 = xyrange(1,3)-1;
    y2 = xyrange(1,4)+1;
end
fill([x1,xyrange(1,1),xyrange(1,1),x1],[y1,y1,y2,y2],[0 0 0],'EdgeColor',[0 0 0]);
fill([x2,xyrange(1,2),xyrange(1,2),x2],[y1,y1,y2,y2],[0 0 0],'EdgeColor',[0 0 0]);
fill([x1,x2,x2,x1],[y1,y1,xyrange(1,3),xyrange(1,3)],[0 0 0],'EdgeColor',[0 0 0]);
fill([x1,x2,x2,x1],[y2,y2,xyrange(1,4),xyrange(1,4)],[0 0 0],'EdgeColor',[0 0 0]);
for k = 1:nObs
    fill(Obs(1,2*sum(sObs(1,1:k-1))+1:2:2*sum(sObs(1,1:k))),Obs(1,2*sum(sObs(1,1:k-1))+2:2:2*sum(sObs(1,1:k))),[0 0 0],'EdgeColor',[0 0 0]);
end
axis([x1,x2,y1,y2]);

% Reference points
plot(x0(1,1),x0(1,2),plotref);
for i = 0:sPath-1
    plot(Con(1,7*Path(i*8+8)-6),Con(1,7*Path(i*8+8)-5),plotref);
end

% Reference path
for i = 0:sPath-1
    if Path(1,i*8+5) < 0  % straight
        plot(linspace(Path(1,i*8+1),Path(1,i*8+3),20),...
             linspace(Path(1,i*8+2),Path(1,i*8+4),20),plotpath);
    else              % curve
        plot(Path(1,i*8+1)*ones(1,20)+Path(1,i*8+5)*cos(linspace(Path(1,i*8+3),Path(1,i*8+4),20)),...
             Path(1,i*8+2)*ones(1,20)+Path(1,i*8+5)*sin(linspace(Path(1,i*8+3),Path(1,i*8+4),20)),plotpath);
    end
end

% Vehicle trajectory
for i = 1:N
    p = plotcar(X(i,1),X(i,2),X(i,3),U(i,2),auto,fig,[0.3 0.3 0.3]);
    pause(0.02)
    if i<N
        delete(p)
    end
    if i<N
        plot([X(i,1),X(i+1,1)],[X(i,2),X(i+1,2)],plottraj);
    end
end

% 2.3) Velocity

figure()
hold on
xlabel('time [s]')
ylabel('v [m/s]')
title('Velocity v')
plot(T,R(1:N,1),plotpath);
plot(T,C(1:N,1),plotcom);
plot(T,U(1:N,1).*U(1:N,3),plottraj);
legend('v_{ref}','v_{com}','v_{act}')

% 2.4) Steering

figure()
hold on
xlabel('time [s]')
ylabel('\delta [deg]')
title('Steering Angle \delta')
plot(T,180/pi*R(1:N,2),plotpath);
plot(T,180/pi*C(1:N,2),plotcom);
plot(T,180/pi*U(1:N,2),plottraj);
legend('\delta_{ref}','\delta_{com}','\delta_{act}')
plot(T,+car.dmax*180/pi*ones(N,1),plotbound);
plot(T,-car.dmax*180/pi*ones(N,1),plotbound);
axis([0,T(size(T,1),1),-car.dmax*180/pi,+car.dmax*180/pi]);


