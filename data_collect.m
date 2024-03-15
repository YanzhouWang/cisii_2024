clear
% warning('off','all')

%%
aurora_device = aurora_bringup_wrapper("/dev/ttyUSB0", 38400); % CHMOD 666

%%
% test speed
t0 = zeros(3,4);
q0 = zeros(4,4);
err0 = zeros(1,4);
em_frames = zeros(4,4,3);

for i = 1:10
    
    tic
    [t0, q0, err0] = aurora_raw_frame(aurora_device);
    disp(t0(:,1))
    time = toc;
    disp(['1: ', num2str(time)])

    % tic
    % em_frames = aurora_get_frames(aurora_device);
    % time = toc;
    % disp(['2: ', num2str(time)])


end


%%
samples = 500;
ntools = 3;
t = zeros(samples,3,ntools);
q = zeros(samples,4,ntools);
err = zeros(samples,ntools);

flag = 0;

for i = 1:samples
    
        % tic
        [t(i,:,:), q(i,:,:), err(i,:)] = aurora_raw_frame(aurora_device);
        
        % time = toc;
        % disp(time)


    % for j = 1:ntools
    %     if i ~= 1 && all((t(i,:,j)-t(i-1,:,j)) < 1e-04)
    %         warning('sensor readings of %d does not change', j)
    %         flag = 1;
    %     end
        % pause(0.0125)
    % end
    % if flag
    %     break
    % end

    if ~mod(i,10)
        disp(i)
    end
end


%%
figure;

subplot(1,3,1)
scatter(t(:,1,1),t(:,2,1))
axis equal
subplot(1,3,2)
scatter(t(:,1,3),t(:,2,2))
axis equal
subplot(1,3,3)
scatter(t(:,1,3),t(:,2,3))
axis equal


%%
sigma1 = zeros(1,3);
sigma1(1) = var(t(:,1,1));
sigma1(2) = var(t(:,2,1));
sigma1(3) = var(t(:,3,1));

sigma2 = zeros(1,3);
sigma2(1) = var(t(:,1,2));
sigma2(2) = var(t(:,2,2));
sigma2(3) = var(t(:,3,2));

sigma3 = zeros(1,3);
sigma3(1) = var(t(:,1,3));
sigma3(2) = var(t(:,2,3));
sigma3(3) = var(t(:,3,3));


sigmaerr1 = var(err(:,1))
sigmaerr2 = var(err(:,2))

meanerr1 = mean(err(:,1))
meanerr2 = mean(err(:,2))

%%

sigma1
sigma2
sigma3

%%
aurora_shutdown_wrapper(aurora_device)