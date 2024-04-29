function [frames,errs] = aurora_get_frames(aurora_device)
n_tools = aurora_device.n_port_handles; % number of tools connected
frames = zeros(4 ,4, n_tools); % 4x4 homogeneous frames
errs = zeros(1,n_tools);

    aurora_device.updateSensorDataAll(); % get new readings
    % Update frames
    % aurora_device.port_handles(2).rot
    for i = 1:n_tools
        % aurora_device.port_handles(i).rot % 5DOF tool rotz == 0 
        frames(1:3, 1:3, i) = quat2rotm(aurora_device.port_handles(i).rot);
        frames(1:4, 4, i) = [aurora_device.port_handles(i).trans'; 1];

        errs(i) = aurora_device.port_handles(i).error;
    end


end