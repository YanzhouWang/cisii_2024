function has_valid_frame = aurora_check_nan(aurora_device)
% runs until there is a valid frame returned by EM tracker
% since in the API, READ_OUT_OF_VOLUME_ALLOWED (0800 or 0801, see API page 
% 15) is always used when calling updateSensorDataAll(), the system will 
% only return NaN frames in the first few frames, and after there is a 
% valid frame, there will be no NaN frames and there is no need to check 
% for them anymore. 

% Therefore, NaN do not need to be checked every time, and this can speed
% up the program (?)

% n_tools = aurora_device.n_port_handles; % number of tools connected
% frames = zeros(4 ,4, n_tools); % 4x4 homogeneous frames

has_valid_frame = false;

counter = 0;
max_counter_before_warning = 10;
max_counter = 50;

for i = 1:max_counter
    aurora_device.updateSensorDataAll(); % get new readings
    % Update frames
    [t, q, err] = aurora_raw_frame(aurora_device);

    nan_flag = anynan([t;q;err]); % check if any element is Nan

    if ~nan_flag
       has_valid_frame = true;
       break;
    end
    

    counter = counter + 1;
    if counter > max_counter_before_warning
        warning('Some frames unreadable. Retrying.')
    end

end



end