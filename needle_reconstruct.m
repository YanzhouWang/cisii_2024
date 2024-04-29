%% Cosserat Rod Real-time FEM
% addpath('../.');
% addpath('./invChol/')
close all
clear
% warning('off','all')

%%
% aurora_device = aurora_bringup_wrapper("/dev/ttyUSB0"); % CHMOD 666

% aurora_device = aurora_bringup_wrapper("/dev/ttyUSB0", 9600); % CHMOD 666
aurora_device = aurora_bringup_wrapper("/dev/ttyUSB0", 230400); % CHMOD 666 c
% sudo chmod 666 /dev/ttyUSB0
% sudo stty -F /dev/ttyUSB0 230400

% ls /dev/ttyUSB* % lsusb
% aurora_device = aurora_bringup_wrapper("/dev/ttyUSB0", 9600); % CHMOD 666 c

%%
for i = 1:5
    [t, ~, err] = aurora_raw_frame(aurora_device);
    disp(t);
    disp(err)
    pause(0.5)
end


%%
F_bm_nb = zeros(4); 
N = 5;
for i = 1:N
    em_frames = aurora_get_frames(aurora_device);
    % frames: 1: world, 2: base marker (bm) not real needle base, 3: needle tip
    % em emitter coord: 0

    % p_0_tip = em_frames(1:3,4,3);
    % F_0_bm = em_frames(:,:,2);
    % F_0_tip = F_0_bm;
    % F_0_tip(1:3,4) = p_0_tip;
    % 
    % % transform back to the true needle base, suppose that length is 150mm
    % F_tip_nb = eye(4); % -150 x
    % F_tip_nb(1,4) = -150;
    % F_0_nb = F_0_tip*F_tip_nb;

    % framedata = invSE3(F_0_bm)*F_0_nb;
    % F_bm_nb = F_bm_nb + framedata;
    
    % norm(p_0_tip - F_0_bm(1:3,4));


    em_frames(:,:,1) = eye(4);
    [F_0_bm,F_0_tip] = base_tip_frames(em_frames, eye(4));

    p_0_tip = F_0_tip(1:3,4);
    F_0_tip = F_0_bm;
    F_0_tip(1:3,4) = p_0_tip;

    % transform back to the true needle base, suppose that length is 150mm
    F_tip_nb = eye(4); % -150 x
    F_tip_nb(1,4) = -150;
    F_0_nb = F_0_tip*F_tip_nb;

    framedata = invSE3(F_0_bm)*F_0_nb;
    F_bm_nb = F_bm_nb + framedata;
    
    % norm(p_0_tip - F_0_bm(1:3,4));

    
    disp(framedata(1:3,4)')
    % disp(norm(p_0_tip - F_0_bm(1:3,4)))
    pause(0.5)
end

F_bm_nb = F_bm_nb / N

[Fwb, Fwt] = base_tip_frames(em_frames,F_bm_nb);


%%
F_bm_nb =[
    1.0000    0.0000    0.0000   75.2956;
    0.0000    1.0000   -0.0000  -31.5101;
    0.0000   -0.0000    1.0000   24.7603;
         0         0         0    1.0000];


Fza = [0 1 0 0; 0 0 1 0; 1 0 0 0; 0 0 0 1]; 
Fy = [-1 0 0 0; 0 1 0 0; 0 0 -1 0; 0 0 0 1];

traj = [];

figure;
global ESC_PRESSED % Tip_FX Tip_FY Tip_FZ Tip_TX Tip_TY Tip_TZ
ESC_PRESSED = 0;
while ~ESC_PRESSED

    [em_frames,err] = aurora_get_frames(aurora_device);

    % % base: rotate wrt y -pi
    % em_frames(:,:,2) = em_frames(:,:,2) * Fy;
    % em_frames(:,:,3) = em_frames(:,:,3) * Fza;

    [F_w_nb,F_w_tip,F_w_bm] = base_tip_frames(em_frames, F_bm_nb, 0);

    traj = [traj, F_w_tip(1:3,4)];

    plot3(traj(1,:), traj(2,:), traj(3,:), 'k-', 'LineWidth', 3);

    hold on

    w_frames = zeros(4,4,2);
    w_frames(:,:,1) = F_w_nb;
    w_frames(:,:,2) = F_w_tip;
    w_frames(:,:,3) = F_w_bm;
    plotTransforms(se3(w_frames),'FrameSize', 10)

    hold off

    dists = em_err_dists(err);

    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    set(gca, 'XDir','reverse')

    title(sprintf('err: w: %.3f; b: %.3f \n dists: w: %.3f; b: %.3f; n: %.3f',...
        err(1),err(2), dists(1), dists(2), dists(3)))

    axis equal
    grid on
    drawnow



    % pause(0.1)
end





%% Initilization
% M: beam properties, F: external wrenches, S: solver properties

M.L = 150; % mm % rod total length % 20
M.Nel = 12; % total number of elements
M.E = 210e3; % MPa % Young modulus 2.1e3 
M.nu = 0.3; % Poisson ratio
M.r = 0.5; % mm % radius of circular beam cross-section % d = 1.05 mm

M.nFrames = M.Nel + 1; % total number of frames
M.frames = zeros(4, 4, M.nFrames); % frames container


Fbt = invSE3(Fwb)*Fwt;
% tbt = Fbt(1:3,4);
% M.d0 = 1/M.Nel*[tbt; 0; 0; 0]; % initial relative configuration

xi = Log_SE3(Fbt);
M.d0 = 1/M.Nel*xi; % initial relative configuration

M.frames = zeros(4, 4, M.nFrames); % frames container

% M.frames(:, :, 1) = Fwb; % initialize first frame
% M.frames(:, :, M.nFrames) = Fwt; % initialize last frame

M.frames(:, :, 1) = eye(4); % initialize first frame
M.frames(:, :, M.nFrames) = Fbt; % initialize last frame


for e = 1:M.Nel - 1
    M.frames(:, :, e + 1) = M.frames(:, :, e)*Exp_SE3(M.d0);
    % M.frames(:, :, e + 1) = M.frames(:, :, e)*[eye(3),M.d0(1:3);0,0,0,1];
end



% M.d0 = M.L/M.Nel*[1; 0; 0; 0; 0; 0]; % initial relative configuration
% M.nFrames = M.Nel + 1; % total number of frames
% M.frames = zeros(4, 4, M.nFrames); % frames container
% M.frames(:, :, 1) = eye(4); % initialize first frame
% for e = 1:M.Nel
%     M.frames(:, :, e + 1) = M.frames(:, :, e)*Exp_SE3(M.d0);
% end
% 



F.dis = zeros(6, 1); % distributed wrench
F.pnt = zeros(6, 1); % concentrated tip wrench

S.fixed_frames = [1, M.Nel + 1]; % array of fixed frames
S.forced_frames = 5; % frame where external wrench is applied
S.tol = 1e-3; % error tolerance

hold on
pltse = plotTransforms(se3(M.frames));
plt3 = plot3(0, 0, 0, 'k-', 'LineWidth', 3);
hold off
A = gca;

fig = gcf;
set(fig, 'WindowKeyPressFcn', @KeyPressCallback);
set(fig, 'Color', 'w')
xlabel('X');
ylabel('Y');
zlabel('Z');
axis equal
grid on





%% Simulation
global ESC_PRESSED % Tip_FX Tip_FY Tip_FZ Tip_TX Tip_TY Tip_TZ
ESC_PRESSED = 0;
% Tip_FX = F.pnt(1); Tip_FY = F.pnt(2); Tip_FZ = F.pnt(3);
% Tip_TX = F.pnt(4); Tip_TY = F.pnt(5); Tip_TZ = F.pnt(6);

while ~ESC_PRESSED && ishghandle(fig)
    % F.pnt = [Tip_FX; Tip_FY; Tip_FZ; Tip_TX; Tip_TY; Tip_TZ];
    % F.pnt = [0;0;0;0;0;0];
    tic

    em_frames = aurora_get_frames(aurora_device);

    [Fwb, Fwt] = base_tip_frames(em_frames,F_bm_nb);

    Fbt = invSE3(Fwb)*Fwt;
    % tbt = Fbt(1:3,4);
    % M.d0 = 1/M.Nel*[tbt; 0; 0; 0]; % initial relative configuration
    % 
    xi = Log_SE3(Fbt);
    M.d0 = 1/M.Nel*xi; % initial relative configuration

    % M.frames = zeros(4, 4, M.nFrames); % frames container
    % M.frames(:, :, 1) = Fwb; % initialize first frame
    % M.frames(:, :, M.nFrames) = Fwt; % initialize last frame


    % 
    M.frames(:, :, 1) = eye(4); % initialize first frame
    M.frames(:, :, M.nFrames) = Fbt; % initialize last frame

    for e = 1:M.Nel - 1
        M.frames(:, :, e + 1) = M.frames(:, :, e)*Exp_SE3(M.d0);
        % M.frames(:, :, e + 1) = M.frames(:, :, e)*[eye(3),M.d0(1:3);0,0,0,1];
    end



    % M.frames(:, :, M.nFrames) = Fbt;

    % tic

    M.frames = Solve(M, F, S);
    t = toc;
    fprintf('Time: %f\n', t);
    Plot(M, A);
end

%%

% M.frames(1:3, 1:3, 13) = eye(3);
% 
% %%
% M.frames = Solve(M, F, S);

aurora_shutdown_wrapper(aurora_device)


%% FEM Solver
function frames = Solve(M, F, S, delta_base)
delta_x = zeros(S.nDOF, 1);
err_mag = 5;

% Since delta_base is purely kinematic, main modification is at the end
frames = M.frames;
frames_converged = M.frames;

nDOF = S.nDOF;
freeDOF = S.freeDOF;
while err_mag > S.tol
    % Main FEM
    K = zeros(nDOF, nDOF); % tangent stiffness matrix
    g_int = zeros(nDOF, 1); % internal force vector
    g_ext = zeros(nDOF, 1); % external force vector
    ds = get_d_from_frames(frames); % current d arrays
    % Loop over each element
    for e = 1:M.Nel
        d_local = ds(:, e);
        P_d_local = P(d_local);

        ke = transpose(P_d_local)*M.K_material*P_d_local/M.l;
        g_int_e = transpose(P_d_local)*M.K_material*(d_local - M.d0)/M.l;
        if ~all(F.dis == 0)
            g_ext_e = int_f_ext(M.l, d_local, F.dis);
        else
            g_ext_e = zeros(12, 1);
        end

        K(S.LM(1:12, e), S.LM(1:12, e)) = K(S.LM(1:12, e), S.LM(1:12, e)) + ke;
        g_int(S.LM(1:12, e)) = g_int(S.LM(1:12, e)) + g_int_e;
        g_ext(S.LM(1:12, e)) = g_ext(S.LM(1:12, e)) + g_ext_e;
    end

    g_ext(6*S.forced_frames - 5: 6*S.forced_frames) = ...
    g_ext(6*S.forced_frames - 5: 6*S.forced_frames) + F.pnt; % external wrench at node

    residual = g_int - g_ext;
    
    delta_x_free = - invChol_mex(K(freeDOF, freeDOF))*residual(freeDOF); % Newton's method
    delta_x(freeDOF) = delta_x(freeDOF) + delta_x_free; % update delta_x
    for n = 1:S.nFrames
        frames(:, :, n) = frames_converged(:, :, n)*Exp_SE3(delta_x(6*n - 5 : 6*n)); % update frames
    end
    err_mag = norm(residual(freeDOF));
    % fprintf('Residual Error: %f\n', err_mag);
    
    %% main modification
    ds = get_d_from_frames(frames); % get information from converged frames
    frames(:, :, 1) = frames(:, :, 1)*delta_base; % increment first frame
    for e = 1:M.Nel % reconstruct the rest
        d_local = ds(:, e);
        frames(:, :, e + 1) = frames(:, :, e)*Exp_SE3(d_local);
    end    
end
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
axis equal
drawnow
end

%% Key Press Function
function KeyPressCallback(~, eventdata)
% global Tip_FX Tip_FY Tip_FZ Tip_TX Tip_TY Tip_TZ
global ESC_PRESSED

key = eventdata.Key;
% if strcmpi(key, 'rightArrow')
%     Tip_FY = Tip_FY + 0.001;
% elseif strcmpi(key, 'leftArrow')
%     Tip_FY = Tip_FY - 0.001;
% elseif strcmpi(key, 'upArrow')
%     Tip_FZ = Tip_FZ + 0.001;
% elseif strcmpi(key, 'downArrow')
%     Tip_FZ = Tip_FZ - 0.001;
% elseif strcmpi(key, 'escape')
if strcmpi(key, 'escape')
    ESC_PRESSED = 1;
end
end