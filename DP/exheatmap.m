% Georg Schildbach, 13/May/2015 --- DP for Parking
% Computes the heat map for the DP-based controller
% --------------------------------------------------------------------------------------------------
% REQUIRES 
% exdynprog.m
% --------------------------------------------------------------------------------------------------
% OUTPUTS
% heatmap to illustrate a set of feasible initial conditions
% --------------------------------------------------------------------------------------------------

clc
clear all
close all
load('dplaw')

% 1) Inputs and Options ----------------------------------------------------------------------------

% 1.1) Grid of Initial Positions

Xhm = linspace(-18,18,37);
Yhm = linspace(5,18,14);
Phm = linspace(0,2*pi*(1-1/12),12);

% 2) Initialization --------------------------------------------------------------------------------

% 2.1) Initialize Variables

xhm = max(size(Xhm));
yhm = max(size(Yhm));
phm = max(size(Phm));
Init = zeros(yhm,xhm);
Succ = zeros(yhm,xhm);
Const(1, 1) = -auto.db;
Const(1, 2) =  auto.w/2;
Const(1, 3) = -auto.db;
Const(1, 4) = -auto.w/2;
Const(1, 5) =  auto.l-auto.db;
Const(1, 6) =  auto.w/2;
Const(1, 7) =  auto.l-auto.db;
Const(1, 8) = -auto.w/2;
Const(1, 9) = (auto.l-auto.df-auto.db)/2;
Const(1,10) =  auto.w/2;
Const(1,11) = (auto.l-auto.df-auto.db)/2;
Const(1,12) = -auto.w/2;
Const(1,13) = -auto.db;
Const(1,14) =  auto.w/4;
Const(1,15) = -auto.db;
Const(1,16) = -auto.w/4;
Const(1,17) =  auto.l-auto.db;
Const(1,18) =  auto.w/4;
Const(1,19) =  auto.l-auto.db;
Const(1,20) = -auto.w/4;
Const(1,21) =  auto.l-auto.db;
Const(1,22) =  0;
Const(1,23) = -auto.db;
Const(1,24) =  0;
Const(1,25) =  auto.l-auto.db-auto.df;
Const(1,26) =  auto.w/2;
Const(1,27) =  auto.l-auto.db-auto.df;
Const(1,28) = -auto.w/2;
Const(1,29) =  0;
Const(1,30) =  auto.w/2;
Const(1,31) =  0;
Const(1,32) = -auto.w/2;
Const(1,33) = -auto.db/2;
Const(1,34) =  auto.w/2;
Const(1,35) = -auto.db/2;
Const(1,36) = -auto.w/2;
Const(1,37) = (auto.l-auto.db)-auto.df/2;
Const(1,38) =  auto.w/2;
Const(1,39) = (auto.l-auto.db)-auto.df/2;
Const(1,40) = -auto.w/2;
Const(1,41) = (auto.l-auto.db-auto.df)/4;
Const(1,42) =  auto.w/2;
Const(1,43) = (auto.l-auto.db-auto.df)/4;
Const(1,44) = -auto.w/2;
Const(1,45) = (auto.l-auto.db-auto.df)*3/4;
Const(1,46) =  auto.w/2;
Const(1,47) = (auto.l-auto.db-auto.df)*3/4;
Const(1,48) = -auto.w/2;

% 2.2) H-Representation of Obstacles

i = 0;
Gobs = zeros(1,2*sum(sObs));
hobs = zeros(1,sum(sObs));
for k = 1:nObs
    j = sObs(1,k);
	for l = 1:j-1
        Gobs(1,2*i+1) = Obs(1,2*i+2) - Obs(1,2*(i+1)+2);
        Gobs(1,2*i+2) = Obs(1,2*(i+1)+1) - Obs(1,2*i+1);
        hobs(1,i+1) = Gobs(1,2*i+1)*Obs(1,2*i+1) + Gobs(1,2*i+2)*Obs(1,2*i+2);
        i = i+1;
    end
    Gobs(1,2*i+1) = Obs(1,2*i+2) - Obs(1,2*(i-l)+2);
	Gobs(1,2*i+2) = Obs(1,2*(i-l)+1) - Obs(1,2*i+1);
	hobs(1,i+1) = Gobs(1,2*i+1)*Obs(1,2*i+1) + Gobs(1,2*i+2)*Obs(1,2*i+2);
	i = i+1;
end

% 3) Computations ----------------------------------------------------------------------------------

for i = 1:xhm
    for j = 1:yhm
        for k = 1:phm
            
            % 3.1) Admissible Initial Condition
            
            coll = 0;
            for l=0:(size(Const,2)/2-1)
                x = Xhm(1,i) + cos(Phm(1,k))*Const(1,2*l+1) - sin(Phm(1,k))*Const(1,2*l+2);
                y = Yhm(1,j) + sin(Phm(1,k))*Const(1,2*l+1) + cos(Phm(1,k))*Const(1,2*l+2);
                if or( x-xyrange(1,1)<1e-10 , x-xyrange(1,2)>-1e-10 )
                	coll = 2;
                end
                if or( y-xyrange(1,3)<1e-10 , y-xyrange(1,4)>-1e-10 )
                	coll = 2;
                end
                oc = 0;
                for o = 1:nObs
                	obscoll = 2;
                    for n = 0:sObs(o)-1
                    	S = Gobs(1,2*oc+1)*x + Gobs(1,2*oc+2)*y - hobs(oc+1);
                        oc = oc+1;
                        if S>=1e-10
                        	obscoll = 0;
                        end
                    end
                    if obscoll > 1
                    	coll = 2;
                    end
                end
            end
            
            % 3.2) Compute Path
            
            if coll < 1
                Init(j,i) = Init(j,i) + 1;
                [Path,sPath] = pathplan([Xhm(1,i),Yhm(1,j),Phm(1,k)],Con,sObs,Obs);
                if sPath >= 0
                    Succ(j,i) = Succ(j,i) + 1;
                end
            end
            
        end
    end
end

% 4) Plot Heat Map ---------------------------------------------------------------------------------

fig = figure();
hold on
axis equal
xlabel('x [m]')
ylabel('y [m]')
title('Heat Map (red: 100% of i.c., blue: 0% of i.c.)')

% 4.1) Obstacles

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

% 4.2) Target Positions

for k=1:sTar
    plotcar(Tar(1,1),Tar(1,2),Tar(1,3),0,auto,fig,[0.5 0.5 0.5])
end

% 4.3) Initial Conditions

for i = 1:xhm
    for j = 1:yhm
        if Init(j,i) > 0
            col = [Succ(j,i)/Init(j,i), 0, 1-Succ(j,i)/Init(j,i)];
            plot(Xhm(1,i),Yhm(1,j),'marker','.','markersize',16,'markeredgecolor',col,'markerfacecolor',col);
        end
    end
end
