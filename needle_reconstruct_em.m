function needle_reconstruct_em()
% Reconstruct needle shape based on EM readings
% Apr 29 2024
% Yanzhou Wang, Junling Mei

addpath(genpath('./helper_funcs'));
close all
clear

%% Aurora and Setup Initialization
% Aurora
aurora_device = aurora_bringup_wrapper("/dev/ttyUSB1", 230400); % CHMOD 666

% Check frames
for i = 1:5
    [tip_trans, ~, err] = aurora_raw_frame(aurora_device);
    disp(tip_trans);
    disp(err)
    pause(0.5)
end

% nb: needle base
% bm: base marker
% w: world

% base marker to needle base, fixed
F_bm_nb =[
    1.0000    0.0000    0.0000   75.2956;
    0.0000    1.0000   -0.0000  -31.5101;
    0.0000   -0.0000    1.0000   24.7603;
    0         0         0    1.0000];

[em_frames, err] = aurora_get_frames(aurora_device);
[F_w_nb, F_w_tip, ~] = base_tip_frames(em_frames, F_bm_nb, 0);

%% Cosserat Initialization
% M: beam properties, F: external wrenches, S: solver properties

%=========================================================================%
% M: tunable parameters
M.L = 150; % rod total length
M.Nel = 40; % total number of elements
M.E = 210; % Young modulus
M.nu = 0.3; % Poisson ratio
M.r = 0.5; % radius of circular beam cross-section
M.d0 = M.L/M.Nel*[1; 0; 0; 0; 0; 0]; % initial relative configuration
M.init_base_frame = F_w_nb;

% F: tunable parameters
F.dis = zeros(6, 1); % distributed wrench
F.pnt = zeros(6, 1); % concentrated tip wrench

% S: tunable parameters
S.EBC_frames = [1, M.Nel + 1]; % array of fixed frames
S.EBC_DOFs = []; % array of fixed DOFs
S.forced_frames = []; % frame where external wrench is applied
S.tol = 1e-3; % error tolerance

%=========================================================================%
% M: automatic
M.l = M.L/M.Nel; % rod element length
M.G = M.E/(2*(1 + M.nu)); % Shear modulus
M.A = pi*M.r^2; % Area of the beam cross-section
M.I = pi*M.r^4/4; % Second moment of area of the beam cross-section
M.J = 2*M.I; % Polar moment of area of the beam cross-section
M.k = 6*(1 + M.nu)/(7 + 6*M.nu); % Shear correction factor
M.K_material = diag(...% material stiffness matrix of cross-section
    [M.E*M.A, M.k*M.G*M.A, M.k*M.G*M.A, ...
    M.G*M.J, M.E*M.I, M.E*M.I]);
M.nFrames = M.Nel + 1; % total number of frames
M.frames = zeros(4, 4, M.nFrames); % frames container
M.frames(:, :, 1) = M.init_base_frame; % initialize first frame
for e = 1:M.Nel
    M.frames(:, :, e + 1) = M.frames(:, :, e)*Exp_SE3(M.d0);
end

% S: automatic
% Construction of LM
Nen = 12; % DOF per element
if ~isfield(S, 'LM')
    S.LM = zeros(Nen, M.Nel);
    for e = 1:M.Nel
        S.LM(:, e) = (6*e - 5) : 6*(e + 1);
    end
end
% Removal of EBC
S.nFrames = M.Nel + 1; % total number of frames
S.DOF = 1:6*S.nFrames; % global DOF number
S.nDOF = length(S.DOF); % total number of DOF

S.freeDOF = S.DOF;
fixed_DOF = [];
if isfield(S, 'EBC_frames') % check fixed frames
    n_EBC_frames = length(S.EBC_frames);
    fixed_idx = zeros(6*n_EBC_frames, 1);
    for i = 1:n_EBC_frames
        fixed_idx(6*i - 5 : 6*i) = ...
            6*S.EBC_frames(i) - 5 : 6*S.EBC_frames(i);
    end
    fixed_DOF = [fixed_DOF; fixed_idx];
end

if isfield(S, 'EBC_DOFs') % check fixed DOFs
    fixed_DOF = [fixed_DOF; S.EBC_DOFs];
end
S.freeDOF(unique(fixed_DOF)) = []; % remove EBC from DOF
%=========================================================================%

%% Figure Initialization
hold on
orig = plotTransforms(se3(M.frames(:, :, 1)));
pltse = plotTransforms(se3(M.frames));
plt3 = plot3(0, 0, 0, 'k-', 'LineWidth', 3);
A = gca;

fig = gcf;
set(fig, 'Color', 'w')
set(fig, 'Position', [3000, 700, 560, 420])
xlabel('X');
ylabel('Y');
zlabel('Z');
axis equal
grid on
Plot(M, A);
end

%% Plotting
function Plot(M, A)
X = squeeze(M.frames(1, 4, :));
Y = squeeze(M.frames(2, 4, :));
Z = squeeze(M.frames(3, 4, :));
set(A.Children(1), 'XData', X, 'YData', Y, 'ZData', Z);
for i = 1:M.nFrames
    A.Children(2).Children(M.nFrames - i + 1).Children.Matrix = ...
        M.frames(:, :, i);
end
drawnow
end