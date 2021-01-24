function [rpy,quat,err_integral]  = MahonyFilter(quat,gyro,accel,err_integral,params)

Kp = params.Kp; % Proportional gain - Unit: 1/seconds. 
                % Roughly: error_dot = - Kp* error when no integral term
                % So, 1/Kp gives the time constant of the error dynamics
                
Ki = params.Ki; % Integral gain - unitless. Used to account for bias in gyro.

% Current orientation estimate
world_R_body = quatToR(quat);
body_R_world = world_R_body';

% Normalise accelerometer measurement
accel = accel / norm(accel);

% Estimated direction of -gravity
g_hat_in_body = body_R_world(:,3); % zhat_world expressed in body

% Orientation angle-axis error is given by the cross product between the 
% "measured" direction and estimated direction of -gravity field
err = cross(accel, g_hat_in_body); 
err_integral = err_integral + err * params.dt;   

% Apply feedback terms
gyro_corrected = gyro + Kp * err + Ki * err_integral;    

% Compute rate of change of quaternion
quaternion_dot = 0.5 * quatProd(quat, [0 gyro_corrected(1) gyro_corrected(2) gyro_corrected(3)]');

% Integrate to yield quaternion
quat = quat + quaternion_dot * params.dt;

quat = quat / norm(quat); % normalise quaternion

rpy = quatToRpy(quat);
