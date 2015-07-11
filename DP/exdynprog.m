% Georg Schildbach, 07/May/2015 --- DP for Parking
% Builds the control policy for particular obstacle configuration via dynamic programming
% --------------------------------------------------------------------------------------------------
% REQUIRES 
% initdynprog.m
% --------------------------------------------------------------------------------------------------
% OUTPUTS
% dplaw: data structure for DP controller, including control law
% --------------------------------------------------------------------------------------------------

clc
clear all
close all
load('dplaw')

% 1) Obstacles for DP (Local Map) ------------------------------------------------------------------

% 1.1) Parallel Parking

Obst = [ ...                 % one obstacle per row, columns list vertex points in clockwise order
    -3.5, 3, -3.5, 0, -15, 0, -15, 3;     % obstacle 1 (x1, y1, x2, y2, ...)
     3.5, 0,  3.5, 3,  15, 3,  15, 0 ...  % obstacle 2 (x1, y1, x2, y2, ...)
    ];
sObst = [4, 4];              % number of obstacle vertex points for each obstacle

% 2) Computation  ----------------------------------------------------------------------------------
 
% 2.1) Map-based Computations

if size(Obst,1) ~= size(sObst,2)
    error('Obstacle information has inconsistent dimensions!')
end
if 2*max(sObst') > size(Obst,2)
    error('Mismatch between number of obstacle points sObst and obstacle points Obst!')
end
if 2*max(sObst') < size(Obst,2)
    Obst = Obst(:,1:max(sObst,1));
end
nObs = size(sObst,2);
sObs = sObst;
Obs = zeros(1,2*sum(sObs));
ind = 0;
for k = 1:nObs
    for j = 1:sObs(1,k)
        Obs(1,2*ind+1:2*ind+2) = Obst(k,(j-1)*2+1:j*2);
        ind = ind+1;
    end
end

% 2.2) Computation

[Con,Xgrid,Ygrid,Pgrid] = dynprog(sObs,Obs);

% 2.3) Optional: Compute Control Law Matrix

if 0
    Conb = zeros(size(Con,2)/7,7);
    for k = 0:size(Conb,1)-1
        Conb(k+1,1:7) = Con(1,k*7+1:k*7+7);
    end
    Conc = zeros(1,6);
    for k = 2:size(Conb,1)
        if Conb(k,4)<2*3
            Conc = [Conc ; [Conb(k,1:4), Conb(k,6:7)]];
        end
    end
    clear Conb
end

% 2.4) Optional: Plot Map with Obstacles and Grid

if 0
    fig = figure()
    hold on
    axis equal
    xlabel('x [m]')
    ylabel('y [m]')
    title('Map (black: obstacles, blue (green): (feasible) reference point')
    plotopt.linestyle = 'none';
    plotopt.marker = '.';
    plotopt.markersize = 8;

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
    
    % All reference points
    plotopt.color = [0 0 1]; % RGB value
    for i = 1:xres(1,1)
        plot(Xgrid(1,i)*ones(1,yres(1,1)),Ygrid(1:yres(1,1)),plotopt);
    end
    for n = 2:nres
        for i = 1:xres(n,1)
            plot(Xgrid(1,sum(xres(1:n-1,1))+i)*ones(1,yres(n,1)),Ygrid(1,sum(yres(1:n-1,1))+1:sum(yres(1:n,1))),plotopt);
        end
    end
    
    % Feasible reference points
    plotopt.color = [0 1 0]; % RGB value
    for i = 1:size(Con,2)/7
        if Con(1,(i-1)*7+4) <= maxarc
            plot(Con(1,(i-1)*7+1),Con(1,(i-1)*7+2),plotopt);
        end
    end
    
end

% 3) Store Data  -----------------------------------------------------------------------------------

save('dplaw','Path','sPath','Con','Xgrid','Ygrid','Pgrid','Obst','Obs','nObs','sObs','dir','origin','M','xyrange','xygrid','xres','yres','pres','nres','Targ','Tar','sTar','maxarc','phitol','condis','auto','car')





