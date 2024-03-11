function aurora_shutdown_wrapper(aurora_device)
aurora_device.stopTracking();
delete(aurora_device);
end