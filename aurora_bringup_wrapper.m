function aurora_device = aurora_bringup_wrapper(USB_Port, BaudRate)
aurora_device = AuroraDriver(USB_Port);
serial_present = instrfind;  % should not be empty if aurora device detected 

% Initialization
try
    aurora_device.openSerialPort();
catch ME
    warning('Port locked. Try issuing sudo chmod %s\n', USB_Port);
end

% edit: baud rate
if nargin == 2
    aurora_device.setBaudRate(BaudRate); % sudo stty -F /dev/ttyUSB0 14400
    disp(['Baud Rate set to ', num2str(BaudRate)])
end

aurora_device.init();
aurora_device.detectAndAssignPortHandles();
aurora_device.initPortHandleAll();
aurora_device.enablePortHandleDynamicAll();

% If everything is successfull you will hear the field generator buzz a bit
% after running this
aurora_device.startTracking();

% Sanity check - make Aurora beep!
aurora_device.BEEP('1');
disp('Connected to NDI Aurora')
end