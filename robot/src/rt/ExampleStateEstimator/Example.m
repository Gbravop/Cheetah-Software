%% Setup

% Filter Paramters
params.Kp = 1;    % Units: 1/seconds. 1/Kp gives the time constant
params.Ki = 0 ;   % Units: 1/seconds. Can be used to account for steady-state bias on gyro
params.dt = 1e-3; % Units: seconds.   Time step between measurements.

% Simulation Paramters
gyro_noise_std_dev = .5;    % Units rad/s
gyro_bias = [.05 .02 .01]'; % Units rad/s
accel_noise_std_dev = .3;    % Units m/s^2
accel_bias = [.05 .02 .01]'; % Units m/s^2

% Integral Error
err_integral = [0 0 0]'; % Unitless

% Initial quaternion and initial estimate
quat_real = [.8 .1 .05 .02]'; quat_real = quat_real/norm(quat_real);
quat_est  = quat_real;

t = 0; % Units: Seconds

% For storing data
t_list = [];
quat_est_list = [];
quat_real_list = [];

rpy_est_list = [];
rpy_real_list = [];


%% Run Simulation of estimator

% Assume IMU rotating purely with noisy gyro and accelerometer readings
while t < 3
    % Some ficticiously assumed angular velocity profile
    omega_real = [ 2*cos(5*t), 3*sin(8*t), 1*cos(3*t) ]'; 
    
    % Assume body is stationary. 
    % You could remote the zero to see how IMU translational accels affect the filter.
    body_acceleration = [2*cos(t) 3*sin(3*t) sin(2*t)]'*0;
    
    world_R_body_real = quatToR(quat_real);
    zhat_body_real    = world_R_body_real(3,:)';
    
    % Mimic noisy sensor readings
    gyro_noise = randn(3,1)*gyro_noise_std_dev;
    accel_noise = randn(3,1)*accel_noise_std_dev;
    
    gyro = omega_real + gyro_noise + gyro_bias;
    accel = zhat_body_real + accel_noise + accel_bias + body_acceleration;
    
    % Run Estimator
    [rpy_est,quat_est,err_integral]  =  ...
                                    MahonyFilter(quat_est,gyro,accel,err_integral,params);
    
    % Integrate true state
    quat_real = quat_real + .5*quatProd(quat_real, [0 ; omega_real])*params.dt;
    quat_real = quat_real/norm(quat_real);
    rpy_real = quatToRpy(quat_real);
    
    % Store data for later plotting
    quat_est_list(:,end+1) = quat_est;
    quat_real_list(:,end+1) = quat_real;
    rpy_est_list(:,end+1) = rpy_est;
    rpy_real_list(:,end+1) = rpy_real;
    t_list(end+1) = t;
    t = t+params.dt;
end

%% Plotting
figure(3); clf;
subplot(311);
h_real(1) = plot(t_list,rpy_real_list(1,:),'LineWidth',3);
hold on;
h_est(1) = plot(t_list,rpy_est_list(1,:),'-.','LineWidth',2);
ylabel('Roll (rad)');
l = legend('Real','Estimated');
g = gca;
g.FontSize = 22;


subplot(312);
h_real(2) = plot(t_list,rpy_real_list(2,:),'LineWidth',3);
hold on;
h_est(2) = plot(t_list,rpy_est_list(2,:),'-.','LineWidth',2);
ylabel('Pitch (rad)');
g = gca;
g.FontSize = 22;

subplot(313);
h_real(3) = plot(t_list,rpy_real_list(3,:),'LineWidth',3);
hold on;
h_est(3) = plot(t_list,rpy_est_list(3,:),'-.','LineWidth',2);

ylabel('Yaw (rad)');

h_est(1).Color = h_real(1).Color;
h_est(2).Color = h_real(2).Color;
h_est(3).Color = h_real(3).Color;
xlabel('Time (s)');

g = gca;
g.FontSize = 22;
