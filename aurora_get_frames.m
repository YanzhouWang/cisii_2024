function frames = aurora_get_frames(aurora_device)
n_tools = aurora_device.n_port_handles; % number of tools connected
frames = zeros(4 ,4, n_tools); % 4x4 homogeneous frames
nan_flag = true;
counter = 0;
max_counter_before_warning = 10;
while nan_flag
    aurora_device.updateSensorDataAll(); % get new readings
    % Update frames
    % aurora_device.port_handles(2).rot
    for i = 1:n_tools
        % aurora_device.port_handles(i).rot % 5DOF tool rotz == 0 
        frames(1:3, 1:3, i) = quat2rotm(aurora_device.port_handles(i).rot);
        frames(1:4, 4, i) = [aurora_device.port_handles(i).trans'; 1];
    end
    nan_flag = anynan(frames); % check if any element is Nan
    counter = counter + 1;
    if counter > max_counter_before_warning
        warning('Some frames unreadable. Retrying.')
    end
end
end