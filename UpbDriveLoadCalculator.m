% Input data - car data

mass = 250 + 80; %kg
trackwidth = 1110; %mm
wheelbase = 1550; %mm
cgHeight = 300; %mm
weightDist = 0.5;
rearRollCenterHeight = 50; %mm
frontRollCenterHeight = 40; %mm

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

ax = 1.5;
ay = 1.5;
az = 0;