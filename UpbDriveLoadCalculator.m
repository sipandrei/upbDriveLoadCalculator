% =========================================================================
% MODEL MATEMATIC DE CALCUL ȘI ÎNFĂȘURĂTOARE FORȚE SUSPENSIE - DR07
% =========================================================================
clear; clc; close all;

% 1. DATE DE INTRARE VEHICUL (PARAMS)
params.mass = 250 + 80;              % kg (masă totală cu pilot)
params.massUnsprung = 16;            % kg (masă nesuspendată)
params.massSprung = params.mass - params.massUnsprung; % kg
params.trackwidth = 1110;            % mm
params.g = 9.806;                    % m/s^2
params.wheelbase = 1550;             % mm
params.cgHeight = 300;               % mm
params.weightDist = 0.5;             % Distribuție masă față
params.rearRollCenterHeight = 50;    % mm
params.frontRollCenterHeight = 40;   % mm
params.tireRadius = 225;             % mm
params.cfw = 1;                      % Rigiditate ruliu față
params.crw = 1;                      % Rigiditate ruliu spate
params.mu = 1.5;                     % Coeficient de frecare pneu-asfalt
params.Downforce = 600;              % N (portanță aerodinamică)
params.PressureCenterDist = 0.6;     % Distribuție aero pe spate (60% spate / 40% față)

% 2. HARDPOINTS GEOMETRIE (GEOM) [x; y; z] în mm (Partea stângă +Y)
geom.FrontUpBack   = [-102.87; 295.31; 267.72];
geom.FrontUpFront  = [127.36;  263.04; 302.05];
geom.FrontUpWheel  = [-9.43;   498.18; 324.36];

geom.FrontLowBack  = [-124.89; 289.62; 124.89];
geom.FrontLowFront = [129.89;  264.20; 135.79];
geom.FrontLowWheel = [9.88;    514.19; 141.41];

geom.FrontTieChassis = [95.00; 229.81; 160.00];
geom.FrontTieWheel   = [80.00; 530.00; 170.00];

geom.FrontActChassis = [3.99;  287.75; 110.89];
geom.FrontActWheel   = [-4.88; 464.30; 290.81];

geom.RearUpBack    = [-131.11 - params.wheelbase; 219.87; 299.21];
geom.RearUpFront   = [164.73  - params.wheelbase; 339.97; 272.80];
geom.RearUpWheel   = [0       - params.wheelbase; 504.52; 331.00];

geom.RearLowBack   = [-134.02 - params.wheelbase; 211.15; 136.74];
geom.RearLowFront  = [128.54  - params.wheelbase; 332.78; 137.60];
geom.RearLowWheel  = [0       - params.wheelbase; 531.20; 141.00];

geom.RearTieChassis  = [-130.00 - params.wheelbase; 215.00; 210.00];
geom.RearTieWheel    = [-70.00  - params.wheelbase; 506.51; 234.13];

geom.RearActWheel    = [32.32 - params.wheelbase; 441.15; 167.80];
geom.RearActChassis  = [43.43 - params.wheelbase; 305.54; 367.52];

geom.FrontContact  = [0; params.trackwidth/2; 0];
geom.RearContact   = [-params.wheelbase; params.trackwidth/2; 0];

% =========================================================================
% 3. DEFINIRE INTERVALE DE ACCELERAȚII ȘI RULARE ÎN BUCLĂ (TASK 4000)
% =========================================================================
ax_range = [-2, -1.5, -1.0, 0.0, 1.0,1.5,2.0, 0.5, -0.5]; % G (negativ = frânare, pozitiv = accelerare)
ay_range = [0.0, 1.0, 1.5, 2, -2, -1.5, -1, 0.5, -0.5];         % G (viraj dreapta)
az_range = [0.0, 1.5];                   % G (0 = pistă plană, 1.5 = trecere peste vibrator/bump)

[AX, AY, AZ] = ndgrid(ax_range, ay_range, az_range);
loadCases = [AX(:), AY(:), AZ(:)];

% Filtrare cazuri nerealiste peste cercul de aderență
validIdx = sqrt(loadCases(:,1).^2 + loadCases(:,2).^2) <= (params.mu * 1.5);
loadCases = loadCases(validIdx, :);
numCases = size(loadCases, 1);

baseMembers = {'UB (Up Back)', 'UF (Up Front)', 'LB (Low Back)', 'LF (Low Front)', 'TR (Tie Rod)', 'AC (Push/Pull)'};
corners = {'FL', 'FR', 'RL', 'RR'};
memberLabels = cell(24, 1);
idx = 1;
for c = 1:4
    for m = 1:6
        memberLabels{idx} = sprintf('%s - %s', corners{c}, baseMembers{m});
        idx = idx + 1;
    end
end

allForcesHistory = zeros(24, numCases);
for k = 1:numCases
    [F_mat, ~, ~] = calculateSuspensionLoads(loadCases(k,1), loadCases(k,2), loadCases(k,3), geom, params);
    allForcesHistory(:, k) = F_mat(:);
end

% Extragere valori maxime de întindere/tracțiune (+), compresiune (-) și solicitare absolută

% Extragere valori maxime de tracțiune (>= 0 N) și compresiune (<= 0 N)
[rawMax, maxTensCaseIdx] = max(allForcesHistory, [], 2);
[rawMin, maxCompCaseIdx] = min(allForcesHistory, [], 2);

maxTension     = max(0, rawMax); % Dacă e doar în compresiune, tracțiunea maximă = 0 N
maxCompression = min(0, rawMin); % Dacă e doar în tracțiune, compresiunea maximă = 0 N

[peakAbsForce, critCaseIdx]          = max(abs(allForcesHistory), [], 2);

% Identificarea maximului global la TRACȚIUNE (+N) și la COMPRESIUNE (-N)
[globalMaxTens, mostTensIdx] = max(maxTension);
[globalMaxComp, mostCompIdx] = min(maxCompression);

% Identificarea celui mai solicitat membru absolut din întreaga mașină
[globalMaxForce, mostLoadedIdx] = max(peakAbsForce);
worstCase = loadCases(critCaseIdx(mostLoadedIdx), :);
signedPeakForce = allForcesHistory(mostLoadedIdx, critCaseIdx(mostLoadedIdx));

fprintf('=========================================================================\n');
fprintf('ANALIZĂ ÎNFĂȘURĂTOARE PENTRU %d CAZURI DE ÎNCĂRCARE (TASK 4000)\n', numCases);
fprintf('MAXIM TRACȚIUNE (+):      %s -> +%.1f N (Caz: [%.2fG, %.2fG, %.2fG])\n', ...
        memberLabels{mostTensIdx}, globalMaxTens, loadCases(maxTensCaseIdx(mostTensIdx), :));
fprintf('MAXIM COMPRESIUNE (-):    %s -> %.1f N (Caz: [%.2fG, %.2fG, %.2fG])\n', ...
        memberLabels{mostCompIdx}, globalMaxComp, loadCases(maxCompCaseIdx(mostCompIdx), :));
fprintf('CEL MAI SOLICITAT MEMBRU: %s -> %.1f N (SF=2.5 -> %.1f N)\n', ...
        memberLabels{mostLoadedIdx}, signedPeakForce, globalMaxForce * 2.5);
fprintf('=========================================================================\n\n');

T_Envelope = table(maxTension, maxCompression, peakAbsForce, peakAbsForce * 2.5, ...
                   'RowNames', memberLabels, ...
                   'VariableNames', {'Max_Tractiune_N', 'Max_Compresiune_N', 'Max_Absolut_N', 'Forta_Calcul_SF2_5_N'});
disp(T_Envelope);

%% ========================================================================
%  4. MODUL DIMENSIONARE BRAȚE (TRACȚIUNE MAXIMĂ & FLAMBAJ EULER)
%  ========================================================================

% --- 4.1 Parametri de intrare (Geometrie țeavă, Material & Multiplicator Dinamic) ---
sizing.Do_mm        = 25.0;     % Diametru exterior tub [mm]
sizing.t_mm         = 1.5;      % Grosime perete tub [mm]
sizing.E_GPa        = 64.0;     % Modul de elasticitate longitudinal (Tensile Modulus) [GPa]
sizing.sigma_ult_t  = 650.0;    % Rezistența la rupere la tracțiune (UTS) [MPa]
sizing.sigma_ult_c  = 450.0;    % Rezistența la compresiune (UCS) [MPa]
sizing.k_dyn        = 1.5;      % Multiplicator dinamic (șocuri / vibrații / borduri) [-]
sizing.K_buckling   = 1.0;      % Factor lungime de flambaj (K = 1.0 pt. articulații sferice / uniball)
sizing.SF_target    = 2.5;      % Factor de siguranță impus (REQ-SYS-02) [-]
sizing.insertOffset = 0.045;    % Lungime totală ocupată de capetele uniball/inserturi la ambele capete [m] (0 pt. distanța între hardpoints)

% --- 4.2 Proprietăți geometrice ale secțiunii inelare ---
Do = sizing.Do_mm * 1e-3;               % [m]
t  = sizing.t_mm  * 1e-3;               % [m]
Di = Do - 2 * t;                        % Diametru interior [m]

if Di <= 0
    error('Eroare geometrie: Grosimea peretelui (2*t) depășește diametrul exterior!');
end

A_sec  = (pi / 4) * (Do^2 - Di^2);      % Aria secțiunii transversale [m^2]
I_sec  = (pi / 64) * (Do^4 - Di^4);     % Momentul de inerție axial [m^4]
E_Pa   = sizing.E_GPa * 1e9;            % Modul Young [Pa]
UTS_Pa = sizing.sigma_ult_t * 1e6;      % [Pa]
UCS_Pa = sizing.sigma_ult_c * 1e6;      % [Pa]

% --- 4.3 Extragerea lungimilor reale ale barelor din structura `geom` (în mm -> m) ---
mirrorY = @(pt) [pt(1); -pt(2); pt(3)];

% Perechile de puncte [Inboard, Outboard] pentru fiecare colț (FL, FR, RL, RR),
% în exact aceeași ordine cu `baseMembers` (UB, UF, LB, LF, TR, AC):
pairs_FL = {
    geom.FrontUpBack,   geom.FrontUpWheel;
    geom.FrontUpFront,  geom.FrontUpWheel;
    geom.FrontLowBack,  geom.FrontLowWheel;
    geom.FrontLowFront, geom.FrontLowWheel;
    geom.FrontTieChassis, geom.FrontTieWheel;
    geom.FrontActChassis, geom.FrontActWheel
};

pairs_FR = {
    mirrorY(geom.FrontUpBack),   mirrorY(geom.FrontUpWheel);
    mirrorY(geom.FrontUpFront),  mirrorY(geom.FrontUpWheel);
    mirrorY(geom.FrontLowBack),  mirrorY(geom.FrontLowWheel);
    mirrorY(geom.FrontLowFront), mirrorY(geom.FrontLowWheel);
    mirrorY(geom.FrontTieChassis), mirrorY(geom.FrontTieWheel);
    mirrorY(geom.FrontActChassis), mirrorY(geom.FrontActWheel)
};

pairs_RL = {
    geom.RearUpBack,   geom.RearUpWheel;
    geom.RearUpFront,  geom.RearUpWheel;
    geom.RearLowBack,  geom.RearLowWheel;
    geom.RearLowFront, geom.RearLowWheel;
    geom.RearTieChassis, geom.RearTieWheel;
    geom.RearActChassis, geom.RearActWheel
};

pairs_RR = {
    mirrorY(geom.RearUpBack),   mirrorY(geom.RearUpWheel);
    mirrorY(geom.RearUpFront),  mirrorY(geom.RearUpWheel);
    mirrorY(geom.RearLowBack),  mirrorY(geom.RearLowWheel);
    mirrorY(geom.RearLowFront), mirrorY(geom.RearLowWheel);
    mirrorY(geom.RearTieChassis), mirrorY(geom.RearTieWheel);
    mirrorY(geom.RearActChassis), mirrorY(geom.RearActWheel)
};

allPairs = [pairs_FL; pairs_FR; pairs_RL; pairs_RR]; % 24 x 2

L_hardpoints_m = zeros(24, 1);
L_bars         = zeros(24, 1);
for i = 1:24
    % Coordonatele din `geom` sunt în mm -> împărțim la 1000 pentru metri
    L_hardpoints_m(i) = norm(allPairs{i, 2} - allPairs{i, 1}) / 1000;
    L_bars(i)         = max(0.05, L_hardpoints_m(i) - sizing.insertOffset);
end

% --- 4.4 Calcul solicitări, forță critică Euler și Factori de Siguranță ---
F_tens_dyn = sizing.k_dyn * maxTension;          % Forță de calcul la tracțiune [N]
F_comp_dyn = sizing.k_dyn * abs(maxCompression); % Forță de calcul la compresiune [N]

% Tensiuni axiale [MPa]
sigma_tens_MPa = (F_tens_dyn / A_sec) * 1e-6;
sigma_comp_MPa = (F_comp_dyn / A_sec) * 1e-6;

% Forța critică de flambaj Euler [N]
F_cr_Euler = (pi^2 * E_Pa * I_sec) ./ ((sizing.K_buckling * L_bars).^2);

% Factori de siguranță
SF_tension  = UTS_Pa ./ (F_tens_dyn / A_sec + eps);
SF_comp_mat = UCS_Pa ./ (F_comp_dyn / A_sec + eps);
SF_buckling = F_cr_Euler ./ (F_comp_dyn + eps);

% Factorul de siguranță critic (minimul dintre toate modurile de cedare)
SF_min = min([SF_tension, SF_comp_mat, SF_buckling], [], 2);

% Determinarea modului critic de cedare pentru fiecare bară
critMode   = cell(24, 1);
statusFlag = cell(24, 1);
for i = 1:24
    [minVal, modeIdx] = min([SF_tension(i), SF_comp_mat(i), SF_buckling(i)]);
    if modeIdx == 1
        critMode{i} = 'Tracțiune';
    elseif modeIdx == 2
        critMode{i} = 'Compresiune (Material)';
    else
        critMode{i} = 'Flambaj Euler';
    end
    
    if minVal >= sizing.SF_target
        statusFlag{i} = 'OK';
    else
        statusFlag{i} = 'SUB LIMITĂ!';
    end
end

% --- 4.5 Afișare Tabel Rezultate Dimensionare ---
SizingTable = table(memberLabels, round(L_hardpoints_m*1000, 1), round(L_bars*1000, 1), ...
    round(F_tens_dyn, 1), round(F_comp_dyn, 1), round(F_cr_Euler, 1), ...
    round(SF_tension, 2),round(SF_comp_mat, 2), round(SF_buckling, 2), round(SF_min, 2), critMode, statusFlag, ...
    'VariableNames', {'Bara', 'L_hardpoints_mm', 'L_tub_mm', 'Ftens_dyn_N', 'Fcomp_dyn_N', ...
    'Fcr_Euler_N', 'SF_Tractiune', 'SF_comp_mat', 'SF_Flambaj', 'SF_Minim', 'Mod_Critic', 'Status'});

fprintf('\n====================================================================================================\n');
fprintf(' DIMENSIONARE STRUCTURALĂ BRAȚE: Tub Ø%.1f x %.1f mm | A = %.3f mm^2 | I = %.3f mm^4 | E = %.1f GPa | k_dyn = %.2f | SF_impus = %.2f\n', ...
    sizing.Do_mm, sizing.t_mm, A_sec*10^6, I_sec*10^12, sizing.E_GPa, sizing.k_dyn, sizing.SF_target);
fprintf('====================================================================================================\n');
disp(SizingTable);

% Identificare cea mai solicitată bară (cel mai mic SF)
[worstSF, worstIdx] = min(SF_min);
fprintf('>> BARA CRITICĂ: %s | SF_minim = %.2f (%s) | Lungime tub = %.1f mm\n', ...
    memberLabels{worstIdx}, worstSF, critMode{worstIdx}, L_bars(worstIdx)*1000);

% =========================================================================
% FIGURE 1: GRAFICUL 3D CU MAȘINA, BRAȚELE (HEATMAP) ȘI TRIADA ACCELERAȚIILOR ÎN CoG
% =========================================================================
[F_crit_mat, ~, tireForcesCrit] = calculateSuspensionLoads(worstCase(1), worstCase(2), worstCase(3), geom, params);
plot3DSuspensionHeatmap(geom, params, F_crit_mat, tireForcesCrit, worstCase);

% =========================================================================
% FIGURE 2: GRAFIC COMPARATIV PE CELE 4 ROȚI (STELUȚĂ TRACȚIUNE MAX & COMPRESIUNE MAX)
% =========================================================================
figure('Name', 'Înfășurătoarea Forțelor Maxime pe Cele 4 Roți', 'Color', 'w', 'Position', [100, 80, 1280, 760]);
sgtitle(sprintf('Forțele Axiale Maxime (%d Cazuri) | Max Tracțiune: %s (+%.0f N) | Max Compresiune: %s (%.0f N)', ...
        numCases, memberLabels{mostTensIdx}, globalMaxTens, memberLabels{mostCompIdx}, globalMaxComp), ...
        'FontSize', 12.5, 'FontWeight', 'bold');

cornerTitles = {'FAȚĂ STÂNGA (FL) - Exterior Viraj', 'FAȚĂ DREAPTA (FR) - Interior Viraj', ...
                'SPATE STÂNGA (RL) - Exterior Viraj', 'SPATE DREAPTA (RR) - Interior Viraj'};

T_mat = reshape(maxTension, [6, 4]);
C_mat = reshape(maxCompression, [6, 4]);
xLimMax = globalMaxForce * 1.35;

hComp = []; hTens = []; hStarComp = []; hStarTens = [];

for c = 1:4
    subplot(2, 2, c);
    hold on; grid on; box on;
    
    yPos = 1:6;
    bComp = barh(yPos, C_mat(:, c), 0.55, 'FaceColor', [0.15, 0.45, 0.85], 'EdgeColor', 'k');
    bTens = barh(yPos, T_mat(:, c), 0.55, 'FaceColor', [0.85, 0.20, 0.20], 'EdgeColor', 'k');
    if c == 1, hComp = bComp; hTens = bTens; end
    
    xline(0, 'k-', 'LineWidth', 1.2);
    
    for m = 1:6
        globalIdx = (c - 1) * 6 + m;
        
        % Etichetă Compresiune
        if C_mat(m, c) < -10
            text(C_mat(m, c) - 0.03 * xLimMax, m, sprintf('%.0f N', C_mat(m, c)), ...
                 'HorizontalAlignment', 'right', 'VerticalAlignment', 'middle', 'FontSize', 8.5, 'FontWeight', 'bold', 'Color', [0.05, 0.25, 0.65]);
        end
        % Etichetă Tracțiune / Întindere
        if T_mat(m, c) > 10
            text(T_mat(m, c) + 0.03 * xLimMax, m, sprintf('+%.0f N', T_mat(m, c)), ...
                 'HorizontalAlignment', 'left', 'VerticalAlignment', 'middle', 'FontSize', 8.5, 'FontWeight', 'bold', 'Color', [0.70, 0.10, 0.10]);
        end
        
        % Steluță Aurie pe COMPRESIUNEA MAXIMĂ din întreaga mașină
        if globalIdx == mostCompIdx
            hStarComp = scatter(globalMaxComp, m, 210, [1.0, 0.80, 0.0], 'p', 'filled', 'MarkerEdgeColor', 'k', 'LineWidth', 1.2);
        end
        
        % Steluță Magenta/Portocalie pe TRACȚIUNEA MAXIMĂ din întreaga mașină
        if globalIdx == mostTensIdx
            hStarTens = scatter(globalMaxTens, m, 210, [1.0, 0.20, 0.85], 'p', 'filled', 'MarkerEdgeColor', 'k', 'LineWidth', 1.2);
        end
    end
    
    set(gca, 'YDir', 'reverse', 'YTick', 1:6, 'YTickLabel', baseMembers, 'FontSize', 9.5, 'FontWeight', 'bold');
    xlim([-xLimMax, xLimMax]);
    xlabel('Forță Axială [N]   (<- Compresiune  |  Tracțiune / Întindere ->)', 'FontSize', 9);
    title(cornerTitles{c}, 'FontSize', 10.5, 'FontWeight', 'bold');
    hold off;
end

% Legendă globală clară plasată sus-centru
lgd = legend([hComp, hTens, hStarComp, hStarTens], ...
             {'Compresiune Maximă (-N)', 'Tracțiune Maximă (+N)', ...
              sprintf('Max Compresiune (%s: %.0f N)', memberLabels{mostCompIdx}, globalMaxComp), ...
              sprintf('Max Tracțiune (%s: +%.0f N)', memberLabels{mostTensIdx}, globalMaxTens)}, ...
             'Orientation', 'horizontal', 'FontSize', 9, 'Box', 'on');
lgd.Position = [0.15, 0.01, 0.70, 0.035];

% =========================================================================
% FUNCȚIA 1: VIZUALIZAREA 3D A SUSPENSIEI, TRIADEI CoG ȘI VECTORILOR ANVELOPEI
% =========================================================================
function plot3DSuspensionHeatmap(geom, params, F_mat, tireForces, caseAcc)
    figure('Name', 'DR07 3D Suspension Geometry & Force Heatmap', 'Color', 'w', 'Position', [80, 60, 1180, 760]);
    hold on; grid on; axis equal;
    
    mirrorY = @(pt) [pt(1); -pt(2); pt(3)];
    plotLink = @(p1, p2, col, lw, style) plot3([p1(1), p2(1)], [p1(2), p2(2)], [p1(3), p2(3)], ...
                                               'Color', col, 'LineStyle', style, 'LineWidth', lw);
    
    % Axe de referință șasiu
    plot3([-params.wheelbase - 200, 300], [0, 0], [0, 0], 'k-.', 'LineWidth', 1.2);
    plot3([0, 0], [-params.trackwidth/2, params.trackwidth/2], [0, 0], 'k:', 'LineWidth', 1);
    plot3([-params.wheelbase, -params.wheelbase], [-params.trackwidth/2, params.trackwidth/2], [0, 0], 'k:', 'LineWidth', 1);
    
    maxF = max(abs(F_mat(:)));
    if maxF == 0, maxF = 1; end
    
    nColors = 256; halfN = nColors / 2;
    customCmap = [linspace(0.05, 0.88, halfN)', linspace(0.25, 0.88, halfN)', linspace(0.85, 0.88, halfN)'; ...
                  linspace(0.88, 0.85, halfN)', linspace(0.88, 0.10, halfN)', linspace(0.88, 0.10, halfN)'];
    colormap(gca, customCmap);
    clim([-maxF, maxF]);
    getForceColor = @(F) interp1(linspace(-maxF, maxF, nColors), customCmap, F, 'nearest', 'extrap');
    
    sides = [1, -1];
    for s = sides
        if s == 1
            fUpB = geom.FrontUpBack; fUpF = geom.FrontUpFront; fUpW = geom.FrontUpWheel;
            fLoB = geom.FrontLowBack; fLoF = geom.FrontLowFront; fLoW = geom.FrontLowWheel;
            fTiC = geom.FrontTieChassis; fTiW = geom.FrontTieWheel;
            fAcC = geom.FrontActChassis; fAcW = geom.FrontActWheel;
            rUpB = geom.RearUpBack; rUpF = geom.RearUpFront; rUpW = geom.RearUpWheel;
            rLoB = geom.RearLowBack; rLoF = geom.RearLowFront; rLoW = geom.RearLowWheel;
            rTiC = geom.RearTieChassis; rTiW = geom.RearTieWheel;
            rAcC = geom.RearActChassis; rAcW = geom.RearActWheel;
            F_front = F_mat(:, 1); F_rear = F_mat(:, 3);
        else
            fUpB = mirrorY(geom.FrontUpBack); fUpF = mirrorY(geom.FrontUpFront); fUpW = mirrorY(geom.FrontUpWheel);
            fLoB = mirrorY(geom.FrontLowBack); fLoF = mirrorY(geom.FrontLowFront); fLoW = mirrorY(geom.FrontLowWheel);
            fTiC = mirrorY(geom.FrontTieChassis); fTiW = mirrorY(geom.FrontTieWheel);
            fAcC = mirrorY(geom.FrontActChassis); fAcW = mirrorY(geom.FrontActWheel);
            rUpB = mirrorY(geom.RearUpBack); rUpF = mirrorY(geom.RearUpFront); rUpW = mirrorY(geom.RearUpWheel);
            rLoB = mirrorY(geom.RearLowBack); rLoF = mirrorY(geom.RearLowFront); rLoW = mirrorY(geom.RearLowWheel);
            rTiC = mirrorY(geom.RearTieChassis); rTiW = mirrorY(geom.RearTieWheel);
            rAcC = mirrorY(geom.RearActChassis); rAcW = mirrorY(geom.RearActWheel);
            F_front = F_mat(:, 2); F_rear = F_mat(:, 4);
        end
        
        frontPairs = {{fUpB, fUpW}, {fUpF, fUpW}, {fLoB, fLoW}, {fLoF, fLoW}, {fTiC, fTiW}, {fAcC, fAcW}};
        rearPairs  = {{rUpB, rUpW}, {rUpF, rUpW}, {rLoB, rLoW}, {rLoF, rLoW}, {rTiC, rTiW}, {rAcC, rAcW}};
        
        for m = 1:6
            p1 = frontPairs{m}{1}; p2 = frontPairs{m}{2}; val = F_front(m);
            plot3([p1(1), p2(1)], [p1(2), p2(2)], [p1(3), p2(3)], '-', 'Color', getForceColor(val), 'LineWidth', 1.8 + 2.5*(abs(val)/maxF));
            if abs(val) > 100
                midPt = (p1 + p2)/2;
                text(midPt(1), midPt(2), midPt(3)+12, sprintf('%.0f N', val), 'FontSize', 7.5, 'FontWeight', 'bold', 'Color', getForceColor(val)*0.75, 'HorizontalAlignment', 'center');
            end
            
            p1 = rearPairs{m}{1}; p2 = rearPairs{m}{2}; val = F_rear(m);
            plot3([p1(1), p2(1)], [p1(2), p2(2)], [p1(3), p2(3)], '-', 'Color', getForceColor(val), 'LineWidth', 1.8 + 2.5*(abs(val)/maxF));
            if abs(val) > 100
                midPt = (p1 + p2)/2;
                text(midPt(1), midPt(2), midPt(3)+12, sprintf('%.0f N', val), 'FontSize', 7.5, 'FontWeight', 'bold', 'Color', getForceColor(val)*0.75, 'HorizontalAlignment', 'center');
            end
        end
        
        plotLink(fLoW, fUpW, [0.45, 0.45, 0.45], 1.5, '--');
        plotLink(rLoW, rUpW, [0.45, 0.45, 0.45], 1.5, '--');
        pts = [fUpB, fUpF, fUpW, fLoB, fLoF, fLoW, fTiC, fTiW, fAcC, fAcW, rUpB, rUpF, rUpW, rLoB, rLoF, rLoW, rTiC, rTiW, rAcC, rAcW];
        scatter3(pts(1,:), pts(2,:), pts(3,:), 28, 'k', 'filled');
    end
    
    cb = colorbar('Location', 'eastoutside');
    cb.Label.String = 'Forță Axială în Braț [N]  (- Compresiune | + Tracțiune)';
    cb.Label.FontWeight = 'bold';
    
    % Triadele de forțe la petele de contact
    contactPoints = [geom.FrontContact, mirrorY(geom.FrontContact), geom.RearContact, mirrorY(geom.RearContact)];
    wheelLabels = {'FL', 'FR', 'RL', 'RR'};
    forceScale = 0.12;
    for w = 1:4
        P = contactPoints(:, w); F = tireForces(:, w) * forceScale;
        scatter3(P(1), P(2), P(3), 55, 'r', 'filled');
        text(P(1)+25, P(2)+sign(P(2))*35, P(3)+20, sprintf('%s (Fx=%.0fN, Fy=%.0fN, Fz=%.0fN)', wheelLabels{w}, tireForces(1,w), tireForces(2,w), tireForces(3,w)), 'FontWeight', 'bold', 'FontSize', 8.5);
        if abs(F(1)) > 1e-3, quiver3(P(1), P(2), P(3), F(1), 0, 0, 0, 'Color', [0.85, 0.1, 0.1], 'LineWidth', 2.0, 'MaxHeadSize', 0.4); end
        if abs(F(2)) > 1e-3, quiver3(P(1), P(2), P(3), 0, F(2), 0, 0, 'Color', [0.1, 0.65, 0.1], 'LineWidth', 2.0, 'MaxHeadSize', 0.4); end
        if abs(F(3)) > 1e-3, quiver3(P(1), P(2), P(3), 0, 0, F(3), 0, 'Color', [0.1, 0.3, 0.85], 'LineWidth', 2.0, 'MaxHeadSize', 0.4); end
    end
    
    % Originea [0,0,0]
    axisLen = 180;
    quiver3(0,0,0, axisLen,0,0,0, 'Color', [0.85,0.1,0.1], 'LineWidth', 2.2);
    quiver3(0,0,0, 0,axisLen,0,0, 'Color', [0.1,0.65,0.1], 'LineWidth', 2.2);
    quiver3(0,0,0, 0,0,axisLen,0, 'Color', [0.1,0.3,0.85], 'LineWidth', 2.2);
    
    % =====================================================================
    % CENTRUL DE GREUTATE (CoG) + TRIADA DE ACCELERAȚII (ax, ay, az)
    % =====================================================================
    cogPos = [-params.wheelbase * (1 - params.weightDist); 0; params.cgHeight];
    plot3([cogPos(1), cogPos(1)], [0, 0], [0, cogPos(3)], 'm--', 'LineWidth', 1.2);
    scatter3(cogPos(1), 0, cogPos(3), 120, [0.85, 0.1, 0.75], 'filled', 'MarkerEdgeColor', 'k');
    text(cogPos(1), 0, cogPos(3) - 35, 'CoG', 'FontWeight', 'bold', 'FontSize', 10, 'Color', [0.65, 0, 0.55], 'HorizontalAlignment', 'center');
    
    accScale = 160; % 1 G = 160 mm vizual pe grafic
    % Săgeată ax (Longitudinal - Roșu)
    if abs(caseAcc(1)) > 1e-3
        quiver3(cogPos(1), 0, cogPos(3), caseAcc(1)*accScale, 0, 0, 0, 'Color', [0.85, 0.1, 0.1], 'LineWidth', 3.0, 'MaxHeadSize', 0.45);
        text(cogPos(1) + caseAcc(1)*accScale, 0, cogPos(3) + 18, sprintf('a_x = %.2f G', caseAcc(1)), ...
             'FontWeight', 'bold', 'FontSize', 9, 'Color', [0.85, 0.1, 0.1], 'HorizontalAlignment', 'center');
    end
    % Săgeată ay (Lateral - Verde)
    if abs(caseAcc(2)) > 1e-3
        quiver3(cogPos(1), 0, cogPos(3), 0, caseAcc(2)*accScale, 0, 0, 'Color', [0.1, 0.65, 0.1], 'LineWidth', 3.0, 'MaxHeadSize', 0.45);
        text(cogPos(1), caseAcc(2)*accScale + sign(caseAcc(2))*35, cogPos(3) + 15, sprintf('a_y = %.2f G', caseAcc(2)), ...
             'FontWeight', 'bold', 'FontSize', 9, 'Color', [0.1, 0.65, 0.1], 'HorizontalAlignment', 'center');
    end
    % Săgeată az (Vertical / Bump - Albastru)
    if abs(caseAcc(3)) > 1e-3
        quiver3(cogPos(1), 0, cogPos(3), 0, 0, caseAcc(3)*accScale, 0, 'Color', [0.1, 0.3, 0.85], 'LineWidth', 3.0, 'MaxHeadSize', 0.45);
        text(cogPos(1), 0, cogPos(3) + caseAcc(3)*accScale + 20, sprintf('a_z = %.2f G', caseAcc(3)), ...
             'FontWeight', 'bold', 'FontSize', 9, 'Color', [0.1, 0.3, 0.85], 'HorizontalAlignment', 'center');
    end
    
    xlabel('X - Longitudinal [mm]'); ylabel('Y - Lateral [mm]'); zlabel('Z - Vertical [mm]');
    title(sprintf('Vizualizare 3D Suspensie DR07 - Cazul Critic [ax=%.1fG, ay=%.1fG, az=%.1fG]', caseAcc(1), caseAcc(2), caseAcc(3)), 'FontSize', 12, 'FontWeight', 'bold');
    view(35, 25); rotate3d on; hold off;
end

% =========================================================================
% FUNCȚIA 2: CALCUL DINAMIC & DESCOMPUNERE 3D PE CELE 4 ROȚI - https://www.sciencedirect.com/science/article/pii/S2666691X20300178
% https://publications.lib.chalmers.se/records/fulltext/239258/239258.pdf 
% =========================================================================
function [F_members_all, M_CNC_all, tireForces_all] = calculateSuspensionLoads(ax_G, ay_G, az_G, geom, params)
    g = params.g; ax = ax_G * g; ay = ay_G * g; az = az_G * g;
    weight = params.mass * g;
    verticalFactor = max(0, 1 + (az / g));
    
    massSprungF   = params.weightDist * params.massSprung;
    massSprungR   = (1 - params.weightDist) * params.massSprung;
    massUnsprungF = params.weightDist * params.massUnsprung;
    massUnsprungR = (1 - params.weightDist) * params.massUnsprung;
    
    cogRollDist = params.cgHeight - (params.frontRollCenterHeight + ...
                  (params.rearRollCenterHeight - params.frontRollCenterHeight) * (1 - params.weightDist));
    
    weightF = params.weightDist * weight * verticalFactor;
    weightR = (1 - params.weightDist) * weight * verticalFactor;
    
    VertFL = weightF/2 + (1 - params.PressureCenterDist) * params.Downforce / 2;
    VertFR = weightF/2 + (1 - params.PressureCenterDist) * params.Downforce / 2;
    VertRL = weightR/2 + params.PressureCenterDist * params.Downforce / 2;
    VertRR = weightR/2 + params.PressureCenterDist * params.Downforce / 2;
    
    stiffRatioF = params.cfw / (params.cfw + params.crw);
    stiffRatioR = params.crw / (params.cfw + params.crw);
    
    deltaVertFL = ay * (massSprungF * params.frontRollCenterHeight / params.trackwidth + ...
                        params.massSprung * cogRollDist / params.trackwidth * stiffRatioF + ...
                        massUnsprungF * params.tireRadius / params.trackwidth);
    deltaVertFR = -deltaVertFL;
    
    deltaVertRL = ay * (massSprungR * params.rearRollCenterHeight / params.trackwidth + ...
                        params.massSprung * cogRollDist / params.trackwidth * stiffRatioR + ...
                        massUnsprungR * params.tireRadius / params.trackwidth);
    deltaVertRR = -deltaVertRL;
    
    verticalLoadRollCenter = [VertFL + deltaVertFL, VertFR + deltaVertFR; VertRL + deltaVertRL, VertRR + deltaVertRR];
    deltaLongTransfer = (params.cgHeight / params.wheelbase) * params.mass * ax / 2;
    verticalTotalLoad = [verticalLoadRollCenter(1,1) - deltaLongTransfer, verticalLoadRollCenter(1,2) - deltaLongTransfer; ...
                         verticalLoadRollCenter(2,1) + deltaLongTransfer, verticalLoadRollCenter(2,2) + deltaLongTransfer];
    
    for i = 1:2
        axleTotalWeight = verticalTotalLoad(i,1) + verticalTotalLoad(i,2);
        if verticalTotalLoad(i,2) < 0
            verticalTotalLoad(i,2) = 0; verticalTotalLoad(i,1) = max(0, axleTotalWeight);
        elseif verticalTotalLoad(i,1) < 0
            verticalTotalLoad(i,1) = 0; verticalTotalLoad(i,2) = max(0, axleTotalWeight);
        end
    end
    
    longLoad = zeros(2,2); latLoad = zeros(2,2);
    totalAcceleration = sqrt(ax^2 + ay^2);
    if totalAcceleration > 1e-6
        for i = 1:2
            for j = 1:2
                longLoad(i,j) = (ax / totalAcceleration) * (params.mu * verticalTotalLoad(i,j));
                latLoad(i,j)  = -(ay / totalAcceleration) * (params.mu * verticalTotalLoad(i,j));
            end
        end
    end
    
    Forces_FL = [longLoad(1,1); latLoad(1,1); verticalTotalLoad(1,1)];
    Forces_FR = [longLoad(1,2); latLoad(1,2); verticalTotalLoad(1,2)];
    Forces_RL = [longLoad(2,1); latLoad(2,1); verticalTotalLoad(2,1)];
    Forces_RR = [longLoad(2,2); latLoad(2,2); verticalTotalLoad(2,2)];
    tireForces_all = [Forces_FL, Forces_FR, Forces_RL, Forces_RR];
    
    mirrorY = @(pt) [pt(1); -pt(2); pt(3)];
    U_FL = computeDirCos([geom.FrontUpBack, geom.FrontUpWheel, geom.FrontUpFront, geom.FrontUpWheel, geom.FrontLowBack, geom.FrontLowWheel, geom.FrontLowFront, geom.FrontLowWheel, geom.FrontTieChassis, geom.FrontTieWheel, geom.FrontActChassis, geom.FrontActWheel]);
    U_RL = computeDirCos([geom.RearUpBack, geom.RearUpWheel, geom.RearUpFront, geom.RearUpWheel, geom.RearLowBack, geom.RearLowWheel, geom.RearLowFront, geom.RearLowWheel, geom.RearTieChassis, geom.RearTieWheel, geom.RearActChassis, geom.RearActWheel]);
    U_FR = computeDirCos([mirrorY(geom.FrontUpBack), mirrorY(geom.FrontUpWheel), mirrorY(geom.FrontUpFront), mirrorY(geom.FrontUpWheel), mirrorY(geom.FrontLowBack), mirrorY(geom.FrontLowWheel), mirrorY(geom.FrontLowFront), mirrorY(geom.FrontLowWheel), mirrorY(geom.FrontTieChassis), mirrorY(geom.FrontTieWheel), mirrorY(geom.FrontActChassis), mirrorY(geom.FrontActWheel)]);
    U_RR = computeDirCos([mirrorY(geom.RearUpBack), mirrorY(geom.RearUpWheel), mirrorY(geom.RearUpFront), mirrorY(geom.RearUpWheel), mirrorY(geom.RearLowBack), mirrorY(geom.RearLowWheel), mirrorY(geom.RearLowFront), mirrorY(geom.RearLowWheel), mirrorY(geom.RearTieChassis), mirrorY(geom.RearTieWheel), mirrorY(geom.RearActChassis), mirrorY(geom.RearActWheel)]);
    
    [F_FL, M_FL] = solveCornerFromDirCos(U_FL, geom.FrontUpWheel, geom.FrontLowWheel, geom.FrontTieWheel, geom.FrontActWheel, geom.FrontContact, Forces_FL, 'upper');
    [F_FR, M_FR] = solveCornerFromDirCos(U_FR, mirrorY(geom.FrontUpWheel), mirrorY(geom.FrontLowWheel), mirrorY(geom.FrontTieWheel), mirrorY(geom.FrontActWheel), mirrorY(geom.FrontContact), Forces_FR, 'upper');
    [F_RL, M_RL] = solveCornerFromDirCos(U_RL, geom.RearUpWheel, geom.RearLowWheel, geom.RearTieWheel, geom.RearActWheel, geom.RearContact, Forces_RL, 'lower');
    [F_RR, M_RR] = solveCornerFromDirCos(U_RR, mirrorY(geom.RearUpWheel), mirrorY(geom.RearLowWheel), mirrorY(geom.RearTieWheel), mirrorY(geom.RearActWheel), mirrorY(geom.RearContact), Forces_RR, 'lower');
    
    F_members_all = [F_FL, F_FR, F_RL, F_RR];
    M_CNC_all     = [M_FL, M_FR, M_RL, M_RR];
end

function U_mat = computeDirCos(pts)
    U_mat = zeros(6, 3);
    for m = 1:6
        vec = pts(:, 2*m - 1) - pts(:, 2*m);
        U_mat(m, :) = (vec / norm(vec))';
    end
end

function [F_members, M_offset_CNC] = solveCornerFromDirCos(U_mat, UpW, LowW, TieW, ActW, ContactPatch, F_tire, actMount)
    u_UB = U_mat(1, :)'; u_UF = U_mat(2, :)'; u_LB = U_mat(3, :)';
    u_LF = U_mat(4, :)'; u_TR = U_mat(5, :)'; u_AC = U_mat(6, :)';
    r_UpW = (UpW - ContactPatch)/1000; r_LowW = (LowW - ContactPatch)/1000; r_TieW = (TieW - ContactPatch)/1000;
    I3 = eye(3); B_upright = [-F_tire; 0; 0; 0];
    if strcmpi(actMount, 'upper')
        A_upright = [u_LB, u_LF, u_TR, I3; cross(r_LowW, u_LB), cross(r_LowW, u_LF), cross(r_TieW, u_TR), cross(r_UpW, I3(:,1)), cross(r_UpW, I3(:,2)), cross(r_UpW, I3(:,3))];
        sol_1 = A_upright \ B_upright; F_LB = sol_1(1); F_LF = sol_1(2); F_TR = sol_1(3); R_Up = sol_1(4:6);
        sol_2 = [u_UB, u_UF, u_AC] \ R_Up; F_UB = sol_2(1); F_UF = sol_2(2); F_AC = sol_2(3);
        r_offset_m = (ActW - UpW) / 1000;
    else
        A_upright = [u_UB, u_UF, u_TR, I3; cross(r_UpW, u_UB), cross(r_UpW, u_UF), cross(r_TieW, u_TR), cross(r_LowW, I3(:,1)), cross(r_LowW, I3(:,2)), cross(r_LowW, I3(:,3))];
        sol_1 = A_upright \ B_upright; F_UB = sol_1(1); F_UF = sol_1(2); F_TR = sol_1(3); R_Low = sol_1(4:6);
        sol_2 = [u_LB, u_LF, u_AC] \ R_Low; F_LB = sol_2(1); F_LF = sol_2(2); F_AC = sol_2(3);
        r_offset_m = (ActW - LowW) / 1000;
    end
    F_members = [F_UB; F_UF; F_LB; F_LF; F_TR; F_AC];
    M_offset_CNC = cross(r_offset_m, F_AC * u_AC);
end

% =========================================================================
% FIGURE 3: SCATTER PLOT OF ACCELERATION LOAD CASES (G-G DIAGRAM & 3D)
% =========================================================================
allCombos = [AX(:), AY(:), AZ(:)];
rejectedCases = allCombos(~validIdx, :);

figure('Name', 'Acceleration Load Cases (G-G Diagram & 3D)', 'Color', 'w', 'Position', [140, 100, 1200, 540]);

% --- Subplot 1: 2D G-G Envelope ---
subplot(1, 2, 1);
hold on; grid on; box on; axis equal;

theta = linspace(0, 2*pi, 300);
plot(params.mu * cos(theta), params.mu * sin(theta), 'g--', 'LineWidth', 1.5, 'DisplayName', sprintf('Nominal Grip (\\mu = %.1f G)', params.mu));
plot((params.mu * 1.5) * cos(theta), (params.mu * 1.5) * sin(theta), 'r-', 'LineWidth', 2.0, 'DisplayName', sprintf('Filter Limit (1.5\\mu = %.2f G)', params.mu * 1.25));

% Rejected points
scatter(rejectedCases(:,2), rejectedCases(:,1), 65, [0.65 0.65 0.65], 'x', 'LineWidth', 1.5, 'DisplayName', 'Filtered Out (> 1.875 G)');

% Valid points split by az
az0  = loadCases(loadCases(:,3) == 0, :);
az15 = loadCases(loadCases(:,3) > 0, :);
scatter(az0(:,2),  az0(:,1),  80, [0.12 0.47 0.71], 'o', 'filled', 'MarkerEdgeColor', 'k', 'DisplayName', 'Valid: Flat Track (a_z = 0 G)');
scatter(az15(:,2), az15(:,1), 90, [1.00 0.50 0.05], '^', 'filled', 'MarkerEdgeColor', 'k', 'DisplayName', 'Valid: Bump/Curb (a_z = 1.5 G)');

xline(0, 'k:', 'HandleVisibility', 'off');
yline(0, 'k:', 'HandleVisibility', 'off');
xlim([-2.5, 2.5]); ylim([-2.5, 2.5]);
xlabel('Lateral Acceleration a_y [G]', 'FontWeight', 'bold');
ylabel('Longitudinal Acceleration a_x [G]', 'FontWeight', 'bold');
title('2D G-G Envelope & Friction Circle Filter', 'FontWeight', 'bold');
legend('Location', 'northwest', 'FontSize', 8.5);
hold off;

% --- Subplot 2: 3D Load Case Space ---
subplot(1, 2, 2);
hold on; grid on; box on;

scatter3(rejectedCases(:,1), rejectedCases(:,2), rejectedCases(:,3), 55, [0.65 0.65 0.65], 'x', 'LineWidth', 1.5, 'DisplayName', 'Filtered Out');
scatter3(az0(:,1),  az0(:,2),  az0(:,3),  80, [0.12 0.47 0.71], 'o', 'filled', 'MarkerEdgeColor', 'k', 'DisplayName', 'Valid (a_z = 0 G)');
scatter3(az15(:,1), az15(:,2), az15(:,3), 90, [1.00 0.50 0.05], '^', 'filled', 'MarkerEdgeColor', 'k', 'DisplayName', 'Valid (a_z = 1.5 G)');

for zVal = [0, 1.5]
    plot3((params.mu * 1.5) * cos(theta), (params.mu * 1.5) * sin(theta), zVal * ones(size(theta)), 'r--', 'LineWidth', 1.2, 'HandleVisibility', 'off');
end

xlim([-2.5, 2.5]); ylim([-2.5, 2.5]); zlim([-2.5, 2.5]);
xlabel('a_x [G]', 'FontWeight', 'bold');
ylabel('a_y [G]', 'FontWeight', 'bold');
zlabel('a_z [G]', 'FontWeight', 'bold');
title('3D Load Case Space (a_x, a_y, a_z)', 'FontWeight', 'bold');
view(-55, 22); rotate3d on;
legend('Location', 'northeast', 'FontSize', 8.5);
hold off;