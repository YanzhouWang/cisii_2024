function [F_w_nb,F_w_tip_comp,F_w_bm] = base_tip_frames(em_frames, F_bm_nb, rot)
% returns needle base frame and compensated tip frame relative to world
% rotation of encoder in rads, around x axis
% frames are in SE3 matrix

    
    % rotate needle tip coordinates such that x axis aligns with needle
    % needle z axis -> x axis
    % Rz(-pi/2) * Ry(-pi/2)
    % x-> y, y-> z, z->x
    % Rz = [0 1 0; -1 0 0; 0 0 1];
    % Ry = [0 0 -1; 0 1 0; 1 0 0];
    % Rza = [0 1 0; 0 0 1; 1 0 0];
    % Fza = [Rza, [0;0;0]; 0 0 0 1]; 

    Frot = [1 0 0 0; 0 cos(-rot) -sin(-rot) 0; 0 sin(-rot) cos(-rot) 0; 0 0 0 1];


    Fza = [0 1 0 0; 0 0 1 0; 1 0 0 0; 0 0 0 1]; 
    Fy = [-1 0 0 0; 0 1 0 0; 0 0 -1 0; 0 0 0 1];


    F_0_tip = em_frames(:,:,3); % qz = qx of F_0_bm?
    F_0_bm = em_frames(:,:,2);
    F_0_w = em_frames(:,:,1);

    % q_0_bm = rotm2quat(F_0_bm(1:3,1:3)); % w x y z    

    F_w_bm = invSE3(F_0_w) * F_0_bm * Fy; % base marker, not base
    % F_w_bm = Fy * invSE3(F_0_w) * F_0_bm; % base marker, not base
    F_w_nb = F_w_bm * F_bm_nb;
    F_w_tip = invSE3(F_0_w) * F_0_tip * Fza; % z transfered to x



    % compensate x rotation using euler angles

    ZYX_t = rotm2eul(F_w_tip(1:3,1:3),'ZYX');
    ZYX_nb = rotm2eul(F_w_nb(1:3,1:3),'ZYX');
    
    ZYX_t_comp = [ZYX_t(1), ZYX_t(2), ZYX_nb(3)];


    R_w_tip_comp = eul2rotm(ZYX_t_comp, 'ZYX');
    
    F_w_tip_comp = F_w_tip;
    F_w_tip_comp(1:3,1:3) = R_w_tip_comp;


    F_w_nb =  F_w_nb * Frot;

    F_w_tip_comp =  F_w_tip_comp * Frot;


end