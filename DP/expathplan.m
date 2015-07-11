% Georg Schildbach, 13/May/2015 --- DP for Parking
% Computes the vehicle path for a particular initial condition
% --------------------------------------------------------------------------------------------------
% REQUIRES 
% exdynprog.m
% --------------------------------------------------------------------------------------------------
% OUTPUTS
% dplaw: data structure for DP controller, including planned path
% --------------------------------------------------------------------------------------------------

clc
clear all
close all
load('dplaw')

% 1) Initialize ------------------------------------------------------------------------------------

% 1.1) Initial condition 

x0 = [8.5 7 0];        % initial condition [x,y,phi]
%x0 = [-8.5 7 5*pi/4]; % initial condition [x,y,phi]
%x0 = [8.5 7 5*pi/4];  % initial condition [x,y,phi]
u0 = [0 0 1];          % initial condition [v,delta,gear]
dt = 0.01;             % simulation sample time

% 1.2) Parameters

n = Inf;                % arcs to go
cost = Inf;             % distance to go
x = x0(1,1);            % vehicle x-coordinate
y = x0(1,2);            % vehicle y-coordinate
phi = x0(1,3);          % vehicle phi-coordinate
v = u0(1,1);            % vehicle speed
delta = u0(1,2);        % steering angle
gear = u0(1,3);         % gear

% 2) Reference Path Construction -------------------------------------------------------------------

% 2.1) Computation

[Path,sPath] = pathplan(x0,Con,sObs,Obs);
if sPath<0
    Path =[];
    display(' ')
    error('No feasible path found!!!')
end

% 2.2) Analysis of Planned Path

for k = 0:sPath-1
    dis = [num2str(k+1,'%d') ') '];
    if Path(k*8+5)<-0.5
        dis = [dis, 'Straight, '];
    else
        if Path(k*8+6)>0
            dis = [dis, 'Left,     '];
        else
            dis = [dis, 'Right,    '];
        end
    end
    if Path(k*8+7)>0
        dis = [dis, 'Forward:  '];
    else
        dis = [dis, 'Backward: '];
    end
    if Path(k*8+6)>=0
        sig = '+';
    else
        sig = '-';
    end
    if Path(k*8+5)<-0.5
        dis = [dis, ' front wheels +00.00 deg,  steering +000.00 deg.'];
    else
        if abs(Path(k*8+6)*180/pi)<10
            dis = [dis, ' front wheels ', sig, '0' num2str(abs(Path(k*8+6)*180/pi),'%.2f'), ' deg, '];
        else
            dis = [dis, ' front wheels ', sig,     num2str(abs(Path(k*8+6)*180/pi),'%.2f'), ' deg, '];
        end
        if abs(Path(k*8+6)*14.5*180/pi)<10
            dis = [dis, ' steering ', sig, '00' num2str(abs(Path(k*8+6)*auto.drat*180/pi),'%.2f'), ' deg.'];
        elseif abs(Path(k*8+6)*14.5*180/pi)<100
            dis = [dis, ' steering ', sig,  '0' num2str(abs(Path(k*8+6)*auto.drat*180/pi),'%.2f'), ' deg.'];
        else
            dis = [dis, ' steering ', sig,      num2str(abs(Path(k*8+6)*auto.drat*180/pi),'%.2f'), ' deg.'];
        end
    end
    disp(dis);
    clear dis sig;
end

% 3) Save Solution ---------------------------------------------------------------------------------

save('dplaw','Path','sPath','Con','Xgrid','Ygrid','Pgrid','Obst','Obs','nObs','sObs','dir','origin','M','xyrange','xygrid','xres','yres','pres','nres','Targ','Tar','sTar','maxarc','phitol','condis','auto','car')


