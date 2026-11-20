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
FrontUpWheel = [-9.43; 498.18; 324.36];

%Front lower arm

FrontLowBack = [-124.89; 289.62; 124.89];
FrontLowFront = [129.89; 264.2; 135.79];
FrontLowWheel = [9.88; 514.19; 141.41];

%Front tie rod

FrontTieChassis = [95; 229.81; 160];
FrontTieWheel = [80; 530; 170];

%Front actuation rod

FrontActChassis = [3.99; 287.75; 110.89];
FrontActWheel = [-4.88; 464.3; 290.81];

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

%Rear actuation rod

RearActChassis = [32.32 - wheelbase; 441.15;167.8];
RearActWheel = [43.43 - wheelbase; 305.54; 367.52];

% Contact points
FrontContact = [0; trackwidth/2; 0];
RearContact = [-wheelbase; trackwidth/2; 0];

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
display(Forces_FR);
display(Forces_RL);
display(Forces_RR);

% 3D VISUALIZATION OF SUSPENSION HARDPOINTS & CONTACT PATCH FORCE TRIADS
% =========================================================================

figure('Name', 'DR07 3D Suspension Geometry & Tire Force Triads', 'Color', 'w', 'Position', [100, 100, 1100, 750]);
hold on; grid on; axis equal;

mirrorY = @(pt) [pt(1); -pt(2); pt(3)];
plotLink = @(p1, p2, col, lw, style) plot3([p1(1), p2(1)], [p1(2), p2(2)], [p1(3), p2(3)], ...
                                           'Color', col, 'LineStyle', style, 'LineWidth', lw);

% Longitudinal centerline (Y = 0, Z = 0)
plot3([-wheelbase - 200, 300], [0, 0], [0, 0], 'k-.', 'LineWidth', 1.2);
% Front and Rear Axle lines
plot3([0, 0], [-trackwidth/2, trackwidth/2], [0, 0], 'k:', 'LineWidth', 1);
plot3([-wheelbase, -wheelbase], [-trackwidth/2, trackwidth/2], [0, 0], 'k:', 'LineWidth', 1);

sides = [1, -1]; % 1 = Left (+Y), -1 = Right (-Y)

for s = sides
    % Apply mirroring if right side (s == -1)
    if s == 1
        fUpB = FrontUpBack;   fUpF = FrontUpFront;   fUpW = FrontUpWheel;
        fLoB = FrontLowBack;  fLoF = FrontLowFront;  fLoW = FrontLowWheel;
        fTiC = FrontTieChassis; fTiW = FrontTieWheel;
        fAcC = FrontActChassis; fAcW = FrontActWheel;
        
        rUpB = RearUpBack;    rUpF = RearUpFront;    rUpW = RearUpWheel;
        rLoB = RearLowBack;   rLoF = RearLowFront;   rLoW = RearLowWheel;
        rTiC = RearTieChassis;  rTiW = RearTieWheel;
        rAcC = RearActChassis;  rAcW = RearActWheel;
    else
        fUpB = mirrorY(FrontUpBack);   fUpF = mirrorY(FrontUpFront);   fUpW = mirrorY(FrontUpWheel);
        fLoB = mirrorY(FrontLowBack);  fLoF = mirrorY(FrontLowFront);  fLoW = mirrorY(FrontLowWheel);
        fTiC = mirrorY(FrontTieChassis); fTiW = mirrorY(FrontTieWheel);
        fAcC = mirrorY(FrontActChassis); fAcW = mirrorY(FrontActWheel);
        
        rUpB = mirrorY(RearUpBack);    rUpF = mirrorY(RearUpFront);    rUpW = mirrorY(RearUpWheel);
        rLoB = mirrorY(RearLowBack);   rLoF = mirrorY(RearLowFront);   rLoW = mirrorY(RearLowWheel);
        rTiC = mirrorY(RearTieChassis);  rTiW = mirrorY(RearTieWheel);
        rAcC = mirrorY(RearActChassis);  rAcW = mirrorY(RearActWheel);
    end

    % --- FRONT SUSPENSION ---
    % Upper Wishbone (Dark Blue)
    plotLink(fUpB, fUpW, [0.00, 0.45, 0.74], 2.5, '-');
    plotLink(fUpF, fUpW, [0.00, 0.45, 0.74], 2.5, '-');
    % Lower Wishbone (Navy / Cyan)
    plotLink(fLoB, fLoW, [0.10, 0.20, 0.55], 2.5, '-');
    plotLink(fLoF, fLoW, [0.10, 0.20, 0.55], 2.5, '-');
    % Upright / Kingpin Axis schematic line (Dashed Gray)
    plotLink(fLoW, fUpW, [0.40, 0.40, 0.40], 1.8, '--');
    % Tie Rod (Orange)
    plotLink(fTiC, fTiW, [0.85, 0.33, 0.10], 2.0, '-');
    % Actuation Rod / Pullrod (Purple)
    plotLink(fAcC, fAcW, [0.49, 0.18, 0.56], 2.2, '-');

    % --- REAR SUSPENSION ---
    % Upper Wishbone (Dark Blue)
    plotLink(rUpB, rUpW, [0.00, 0.45, 0.74], 2.5, '-');
    plotLink(rUpF, rUpW, [0.00, 0.45, 0.74], 2.5, '-');
    % Lower Wishbone (Navy / Cyan)
    plotLink(rLoB, rLoW, [0.10, 0.20, 0.55], 2.5, '-');
    plotLink(rLoF, rLoW, [0.10, 0.20, 0.55], 2.5, '-');
    % Rear Upright schematic line (Dashed Gray)
    plotLink(rLoW, rUpW, [0.40, 0.40, 0.40], 1.8, '--');
    % Rear Tie Rod (Orange)
    plotLink(rTiC, rTiW, [0.85, 0.33, 0.10], 2.0, '-');
    % Rear Actuation Rod (Purple)
    plotLink(rAcC, rAcW, [0.49, 0.18, 0.56], 2.2, '-');

    % Plot Joint Nodes (Hardpoints)
    pts = [fUpB, fUpF, fUpW, fLoB, fLoF, fLoW, fTiC, fTiW, fAcC, fAcW, ...
           rUpB, rUpF, rUpW, rLoB, rLoF, rLoW, rTiC, rTiW, rAcC, rAcW];
    scatter3(pts(1,:), pts(2,:), pts(3,:), 30, 'k', 'filled');
end

% -------------------------------------------------------------------------
% 3. PLOT CONTACT PATCH FORCE TRIADS (Fx, Fy, Fz) FOR ALL 4 WHEELS
% -------------------------------------------------------------------------
% Contact patch coordinates [FL, FR, RL, RR]
cp_FL = FrontContact;
cp_FR = mirrorY(FrontContact);
cp_RL = RearContact;
cp_RR = mirrorY(RearContact);

contactPoints = [cp_FL, cp_FR, cp_RL, cp_RR];
allForces     = [Forces_FL, Forces_FR, Forces_RL, Forces_RR];
wheelLabels   = {'FL', 'FR', 'RL', 'RR'};

% Scale factor to convert Newtons (N) into millimeters (mm) for clear arrow sizing
% Example: 2000 N * 0.15 = 300 mm arrow length
forceScale = 0.15; 

for w = 1:4
    P = contactPoints(:, w);
    F = allForces(:, w) * forceScale;
    
    % Mark contact patch center
    scatter3(P(1), P(2), P(3), 60, 'r', 'filled');
    text(P(1) + 30, P(2) + (sign(P(2))*40), P(3) + 20, ...
         sprintf('%s (Fx=%.0fN, Fy=%.0fN, Fz=%.0fN)', wheelLabels{w}, allForces(1,w), allForces(2,w), allForces(3,w)), ...
         'FontWeight', 'bold', 'FontSize', 9);
    
    % Plot Fx arrow (Red - Longitudinal)
    if abs(F(1)) > 1e-3
        hFx = quiver3(P(1), P(2), P(3), F(1), 0, 0, 0, 'Color', [0.85, 0.1, 0.1], 'LineWidth', 2.2, 'MaxHeadSize', 0.4);
    end
    % Plot Fy arrow (Green - Lateral)
    if abs(F(2)) > 1e-3
        hFy = quiver3(P(1), P(2), P(3), 0, F(2), 0, 0, 'Color', [0.1, 0.65, 0.1], 'LineWidth', 2.2, 'MaxHeadSize', 0.4);
    end
    % Plot Fz arrow (Blue - Vertical)
    if abs(F(3)) > 1e-3
        hFz = quiver3(P(1), P(2), P(3), 0, 0, F(3), 0, 'Color', [0.1, 0.3, 0.85], 'LineWidth', 2.2, 'MaxHeadSize', 0.4);
    end
end

% -------------------------------------------------------------------------
% 4. FORMATTING, LABELS & LEGEND
% -------------------------------------------------------------------------
xlabel('X - Longitudinal [mm] (+ Front)');
ylabel('Y - Lateral [mm] (+ Left)');
zlabel('Z - Vertical [mm] (+ Up)');
title('DR07 3D Suspension Geometry & Contact Patch Force Triads', 'FontSize', 13, 'FontWeight', 'bold');

% Dummy lines for clean legend
p1 = plot3(nan, nan, nan, '-', 'Color', [0.00, 0.45, 0.74], 'LineWidth', 2.5);
p2 = plot3(nan, nan, nan, '-', 'Color', [0.10, 0.20, 0.55], 'LineWidth', 2.5);
p3 = plot3(nan, nan, nan, '-', 'Color', [0.85, 0.33, 0.10], 'LineWidth', 2.0);
p4 = plot3(nan, nan, nan, '-', 'Color', [0.49, 0.18, 0.56], 'LineWidth', 2.2);
p5 = plot3(nan, nan, nan, '-', 'Color', [0.85, 0.10, 0.10], 'LineWidth', 2.2);
p6 = plot3(nan, nan, nan, '-', 'Color', [0.10, 0.65, 0.10], 'LineWidth', 2.2);
p7 = plot3(nan, nan, nan, '-', 'Color', [0.10, 0.30, 0.85], 'LineWidth', 2.2);

legend([p1, p2, p3, p4, p5, p6, p7], ...
       {'Upper Wishbone', 'Lower Wishbone', 'Tie Rod', 'Actuation Rod (Push/Pull)', ...
        'F_x (Longitudinal)', 'F_y (Lateral)', 'F_z (Vertical)'}, ...
       'Location', 'northeastoutside');

% =========================================================================
% 5. PLOT GLOBAL ORIGIN TRIAD [0,0,0] & CoG ACCELERATION VECTORS
% =========================================================================

% 1. Global Coordinate System Triad at [0, 0, 0] (Front Axle Center, Ground)
origin = [0; 0; 0];
axisLen = 220; % Length of origin triad arrows in mm

scatter3(origin(1), origin(2), origin(3), 70, 'k', 'filled');
text(origin(1) + 20, origin(2) - 40, origin(3) - 20, 'Origin [0,0,0]', ...
     'FontWeight', 'bold', 'FontSize', 9, 'Color', 'k');

% +X Axis (Forward - Red)
quiver3(0, 0, 0, axisLen, 0, 0, 0, 'Color', [0.85, 0.1, 0.1], 'LineWidth', 3.0, 'MaxHeadSize', 0.35);
text(axisLen + 25, 0, 0, '+X (Front)', 'Color', [0.85, 0.1, 0.1], 'FontWeight', 'bold', 'FontSize', 9);

% +Y Axis (Left - Green)
quiver3(0, 0, 0, 0, axisLen, 0, 0, 'Color', [0.1, 0.65, 0.1], 'LineWidth', 3.0, 'MaxHeadSize', 0.35);
text(0, axisLen + 25, 0, '+Y (Left)', 'Color', [0.1, 0.65, 0.1], 'FontWeight', 'bold', 'FontSize', 9);

% +Z Axis (Up - Blue)
quiver3(0, 0, 0, 0, 0, axisLen, 0, 'Color', [0.1, 0.3, 0.85], 'LineWidth', 3.0, 'MaxHeadSize', 0.35);
text(0, 0, axisLen + 25, '+Z (Up)', 'Color', [0.1, 0.3, 0.85], 'FontWeight', 'bold', 'FontSize', 9);

% 2. Center of Gravity (CoG) Position & Acceleration Triad
% X_cg is measured backward from front axle: -wheelbase * (1 - weightDist)
cogPos = [-wheelbase * (1 - weightDist); 0; cgHeight];

% Vertical drop-line from CoG to ground plane (Z = 0) for spatial clarity
plot3([cogPos(1), cogPos(1)], [cogPos(2), cogPos(2)], [0, cogPos(3)], ...
      'm--', 'LineWidth', 1.4);

% Plot CoG marker
scatter3(cogPos(1), cogPos(2), cogPos(3), 110, [0.85, 0.1, 0.75], 'filled', 'MarkerEdgeColor', 'k');
text(cogPos(1) + 30, cogPos(2) + 30, cogPos(3) + 35, ...
     sprintf('CoG [%.0f, 0, %.0f]', cogPos(1), cogPos(3)), ...
     'FontWeight', 'bold', 'FontSize', 9.5, 'Color', [0.65, 0.0, 0.55]);

% Scale factor to convert m/s^2 into mm (e.g., 1.5*g = 14.71 m/s^2 * 18 = ~265 mm)
accelScale = 18; 

% Longitudinal Acceleration (ax) at CoG (Magenta)
if abs(ax) > 1e-3
    quiver3(cogPos(1), cogPos(2), cogPos(3), ax * accelScale, 0, 0, 0, ...
            'Color', [0.85, 0.0, 0.65], 'LineWidth', 2.8, 'MaxHeadSize', 0.35);
    text(cogPos(1) + ax * accelScale, cogPos(2), cogPos(3) + 20, ...
         sprintf('a_x = %.2fG', ax/g), 'Color', [0.85, 0.0, 0.65], 'FontWeight', 'bold', 'FontSize', 9);
end

% Lateral Acceleration (ay) at CoG (Cyan/Teal)
if abs(ay) > 1e-3
    quiver3(cogPos(1), cogPos(2), cogPos(3), 0, ay * accelScale, 0, 0, ...
            'Color', [0.0, 0.65, 0.75], 'LineWidth', 2.8, 'MaxHeadSize', 0.35);
    text(cogPos(1), cogPos(2) + ay * accelScale + 20, cogPos(3), ...
         sprintf('a_y = %.2fG', ay/g), 'Color', [0.0, 0.65, 0.75], 'FontWeight', 'bold', 'FontSize', 9);
end

% Vertical Acceleration (az) at CoG (Dark Amber)
if abs(az) > 1e-3
    quiver3(cogPos(1), cogPos(2), cogPos(3), 0, 0, az * accelScale, 0, ...
            'Color', [0.85, 0.55, 0.0], 'LineWidth', 2.8, 'MaxHeadSize', 0.35);
    text(cogPos(1), cogPos(2), cogPos(3) + az * accelScale + 20, ...
         sprintf('a_z = %.2fG', az/g), 'Color', [0.85, 0.55, 0.0], 'FontWeight', 'bold', 'FontSize', 9);
end

view(35, 25); % Isometric 3D viewing angle
rotate3d on;  % Enable interactive mouse rotation
hold off;

dirCosMat = [];

% Decomposing force to rigid structural members https://www.sciencedirect.com/science/article/pii/S2666691X20300178 
% =========================================================================
for s = sides
    % Apply mirroring if right side (s == -1)
    if s == 1
        fUpB = FrontUpBack;   fUpF = FrontUpFront;   fUpW = FrontUpWheel;
        fLoB = FrontLowBack;  fLoF = FrontLowFront;  fLoW = FrontLowWheel;
        fTiC = FrontTieChassis; fTiW = FrontTieWheel;
        fAcC = FrontActChassis; fAcW = FrontActWheel;
        
        rUpB = RearUpBack;    rUpF = RearUpFront;    rUpW = RearUpWheel;
        rLoB = RearLowBack;   rLoF = RearLowFront;   rLoW = RearLowWheel;
        rTiC = RearTieChassis;  rTiW = RearTieWheel;
        rAcC = RearActChassis;  rAcW = RearActWheel;
    else
        fUpB = mirrorY(FrontUpBack);   fUpF = mirrorY(FrontUpFront);   fUpW = mirrorY(FrontUpWheel);
        fLoB = mirrorY(FrontLowBack);  fLoF = mirrorY(FrontLowFront);  fLoW = mirrorY(FrontLowWheel);
        fTiC = mirrorY(FrontTieChassis); fTiW = mirrorY(FrontTieWheel);
        fAcC = mirrorY(FrontActChassis); fAcW = mirrorY(FrontActWheel);
        
        rUpB = mirrorY(RearUpBack);    rUpF = mirrorY(RearUpFront);    rUpW = mirrorY(RearUpWheel);
        rLoB = mirrorY(RearLowBack);   rLoF = mirrorY(RearLowFront);   rLoW = mirrorY(RearLowWheel);
        rTiC = mirrorY(RearTieChassis);  rTiW = mirrorY(RearTieWheel);
        rAcC = mirrorY(RearActChassis);  rAcW = mirrorY(RearActWheel);
    end

    % Structural members pairs
    pts = [fUpB, fUpW, fUpF, fUpW, fLoB, fLoW, fLoF, fLoW, fTiC, fTiW, fAcC, fAcW, ...
           rUpB, rUpW, rUpF, rUpW, rLoB, rLoW, rLoF, rLoW, rTiC, rTiW, rAcC, rAcW];
    i = 2;
    while i <= size(pts,2)
        chassisPoint = pts(1:3,i-1);
        wheelPoint = pts(1:3,i);
        mag = norm(chassisPoint-wheelPoint);
        dirCosMat = [dirCosMat, (chassisPoint-wheelPoint)/mag];
        i = i+2;
    end
end

% cosines of structural members - ux, uy, uz
% member orders Up arm back, Up arm front, Low arm Back, Low arm front,
% Tie rod, actuation arm
frontLWheelU = transpose(dirCosMat(1:3, 1:6)); 
rearLWheelU = transpose(dirCosMat(1:3, 7:12));
frontRWheelU = transpose(dirCosMat(1:3, 13:18));
rearRWheelU = transpose(dirCosMat(1:3, 19:24));