%        ___^____
%   ---  |      |  ---
%   |F|  |FRONT |  |F|
%   |L|  |      |  |R| 
%   ---  |      |  ---
%        |      |
%   ---  |      |  ---
%   |R|  |      |  |R|
%   |L|  |______|  |R|
%   ---            ---
% Input data - car data

mass = 250 + 80; %kg
massUsprung = 16; %kg - ASSUMPTION
massSprung = mass - massUsprung; %kg 
trackwidth = 1110; %mm
g = 9.806; %m/s^2
weight = mass*g;
wheelbase = 1550; %mm
cgHeight = 300; %mm
weightDist = 0.5; %Front/Rear
rearRollCenterHeight = 50; %mm
frontRollCenterHeight = 40; %mm
cogRollDist = cgHeight - (frontRollCenterHeight + (rearRollCenterHeight - frontRollCenterHeight) * (1-weightDist)); % mm
cfw = 1; %rad*s^2/m - keep equal - assumption
crw = 1; %rad*s^2/m - keep equal - assumption

%Input data - pick up points [x;y;z] x - long, z - up, positive is towards front

%Front - from center of front wheel

%Front upper arm

FrontUpBack = [-102.87; 295.31; 267.72];
FrontUpFront = [127.36; 263.04; 302.05];
FrontUpWheel = [-9.43; 298.18; 324.36];

%Front lower arm

FrontLowBack = [-124.89; 289.62; 124.89];
FrontLowFront = [129.89; 264.2; 135.79];
FrontLowWheel = [9.88; 514.19; 141.41];

%Front tie rod

FrontTieChassis = [95; 229.81; 160];
FrontTieWheel = [80; 530; 170];

%Rear - from center of rear wheel

%Rear upper arm

RearUpBack = [-131.11 - wheelbase; 219.87; 299.21];
RearUpFront = [164.73 - wheelbase; 339.97; 272.8];
RearUpWheel = [0 - wheelbase; 504.52; 331];

%Rear lower arm

RearLowBack = [-134.02 - wheelbase; 211.15; 136.74];
RearLowFront = [128.54 - wheelbase; 332.78; 137.6];
RearLowWheel = [0 - wheelbase; 531.2; 141];

%Rear tie rod

RearTieChassis = [-130 - wheelbase; 215; 210];
RearTieWheel = [-70 - wheelbase; 506.51; 234.13];

% Input data - known acceleration

ax = -1.5*g; %m/s^2 - this is 
ay = 1.5*g; %m/s^2
az = 0*g;  %m/s^2

% Vertical Forces

weightF = weightDist*weight;
weightR = (1-weightDist)*weight;
massF = weightDist*mass;
massR = (1-weightDist)*mass;
VertFL = weightF/2; % static weight per wheel
VertFR = weightF/2;
VertRL = weightR/2;
VertRR = weightR/2;
verticalLoad = [VertFL, VertFR; VertRL, VertRR];
disp(verticalLoad);
% Incarcari pe roti - steady state

% Transfer lateral

verticalLoadNoRollCenter = [verticalLoad(1,1)+massF*ay*cgHeight/trackwidth,verticalLoad(1,2)-massF*ay*cgHeight/trackwidth; ...
                verticalLoad(2,1)+massR*ay*cgHeight/trackwidth, verticalLoad(2,2)-massR*ay*cgHeight/trackwidth];

disp(verticalLoadNoRollCenter);

%  Lateral Transfer - geometric + stiffness
% https://publications.lib.chalmers.se/records/fulltext/239258/239258.pdf -
% force overestimation - we take all mass, not just unsprung

deltaVertFL = ay*(massF*frontRollCenterHeight/trackwidth+mass*cogRollDist/trackwidth*cfw/(cfw+crw)); % lateral weight transfer taking into account the roll center height
deltaVertFR = -deltaVertFL;
deltaVertRL = ay*(massR*rearRollCenterHeight/trackwidth+mass*cogRollDist/trackwidth*crw/(cfw+crw));
deltaVertRR = -deltaVertRL;

verticalLoadRollCenter = [verticalLoad(1,1)+deltaVertFL,verticalLoad(1,2)+deltaVertFR; ...
                verticalLoad(2,1)+deltaVertRL, verticalLoad(2,2)+deltaVertRR];

disp(verticalLoadRollCenter);

deltaLongTransfer = cgHeight/wheelbase*mass*ax/2;
verticalTotalLoad = [verticalLoadRollCenter(1,1)-deltaLongTransfer, verticalLoadRollCenter(1,2)-deltaLongTransfer;...
                     verticalLoadRollCenter(2,1)+deltaLongTransfer, verticalLoadRollCenter(2,2)+deltaLongTransfer];
for i = 1:2 % i=1 front wheels, i=2 rear wheels
    axleTotalWeight = verticalTotalLoad(i,1) + verticalTotalLoad(i,2);
    if verticalTotalLoad(i,2) < 0
        verticalTotalLoad(i,2) = 0;
        verticalTotalLoad(i,1) = axleTotalWeight;
    elseif verticalTotalLoad(i,1) < 0
        verticalTotalLoad(i,1) = 0;
        verticalTotalLoad(i,2) = axleTotalWeight;
    end
end

disp(verticalTotalLoad);

% Longitudinal and lateral tire forces

mu = 1.5; %friction coefficient - assumption now - MUST GET FROM PACEJKA
longLoad = [0,0;0,0];
latLoad = [0,0;0,0];

totalAcceleration = sqrt(ax^2+ay^2);
if totalAcceleration > 0
    for i = 1:2
        for j = 1:2
            longLoad(i,j) = (ax/totalAcceleration)*(mu*verticalTotalLoad(i,j));
            latLoad(i,j) = -(ay/totalAcceleration)*(mu*verticalTotalLoad(i,j));
        end
    end
end

disp(longLoad);
disp(latLoad);

tireForces = cat(3, longLoad, latLoad, verticalTotalLoad);

%  extract [Fx; Fy; Fz] column vector for any wheel (e.g., Front-Left: i=1, j=1):
Forces_FL = squeeze(tireForces(1, 1, :)); % Returns 3x1 vector [Fx; Fy; Fz]
Forces_FR = squeeze(tireForces(1, 2, :));
Forces_RL = squeeze(tireForces(2, 1, :));
Forces_RR = squeeze(tireForces(2, 2, :));

display(Forces_FL);