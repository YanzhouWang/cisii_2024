function [t, q, err] = aurora_raw_frame(aurora_device)

% if with nan check, 0.09s - 9600


% since all sensor datas are updated, for speed, will return data for all
% tools

% returns: t: 3*ntools, q: 4*ntools, err* ntools
% 
% nan_flag = true;
% counter = 0;
% max_counter_before_warning = 10;

n_tools = aurora_device.n_port_handles; % number of tools connected

t = ones(3,n_tools);
q = zeros(4,n_tools);
err = zeros(1,n_tools);

% while nan_flag
    aurora_device.updateSensorDataAll(); % get new readings
    % Update frames
    % aurora_device.port_handles(2).rot

    % aurora_device.port_handles(i).rot % 5DOF tool rotz == 0 
    for i = 1:n_tools
        
        t(:,i) = aurora_device.port_handles(i).trans';
        q(:,i) = aurora_device.port_handles(i).rot';
        err(i) = aurora_device.port_handles(i).error;
    end

    % nan_flag = anynan([t;q;err]); % check if any element is Nan
    % counter = counter + 1;
%     if counter > max_counter_before_warning
%         warning('Some frames unreadable. Retrying.')
%     end
% end

% counter

end
