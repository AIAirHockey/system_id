function [error] = CoreXY_analysis_func(x, filename)

PULLEY_RADIUS = 3.5686666463209935;

noLoadCurrent = 2.1; % Amp
noLoadSpeed = 5600; % RPM
noLoadSpeed_rad = noLoadSpeed * 2*pi / 60; % rad/s
Km = 0.04025; % Nm / Amp

R = 0.19; % 0.3; % Ohms
L = 188e-6; % Henrys

J_X = x(1)/ 1000 / 100^2;
J_Y = x(2)/ 1000 / 100^2;
J_theta = x(3)/ 1000 / 100^2;
B_X_coeff = x(4);
B_Y_coeff = x(5);

Jm = 3.5^2/4.*[J_X+J_Y J_Y-J_X; J_Y-J_X J_X+J_Y] + J_theta*eye(2);
J_inv = inv(Jm);

B_X = B_X_coeff *noLoadCurrent * Km / noLoadSpeed_rad; % Nms
B_Y = B_Y_coeff*noLoadCurrent * Km / noLoadSpeed_rad; % Nms

Bm = [B_X 0; 0 B_Y];

A = zeros(6);
A(1:2,:) = [0 0 1 0 0 0; 0 0 0 1 0 0;];
A(3:4,:) = [zeros(2), -J_inv*Bm, J_inv*Km];
A(5:6,:) = [zeros(2), -Km/L * eye(2), -R/L * eye(2)];
B = [zeros(4,2); eye(2)*1/L];
C = [eye(2) zeros(2,4)];
D = zeros(2);

% k_p = 1.0; % 15;
% k_i = 0; %10;
% k_d = 0.0003; %.5;

Kff = minreal(inv(tf(ss(A,B,C,D)))).numerator;

Kff{1,1} = Kff{1,1};
Kff{1,2} = Kff{1,2};
Kff{2,1} = Kff{2,1};
Kff{2,2} = Kff{2,2};

theta_rad_to_xy_cm = [1,-1;1,1]*PULLEY_RADIUS/2;

load(filename, 'data');

model = 'CoreXY_model_PWM';
simIn = Simulink.SimulationInput(model);
simIn = setVariable(simIn,'data',data);
simIn = setVariable(simIn, 'theta_rad_to_xy_cm',theta_rad_to_xy_cm);
simIn = setVariable(simIn, 'A',A);
simIn = setVariable(simIn, 'B',B);
simIn = setVariable(simIn, 'C',C);
simIn = setVariable(simIn, 'D',D);
out = sim(simIn);
% out = sim(simIn,'SaveOutput','on','OutputSaveName','yout','SaveFormat','Dataset');

resampledSim = resample(timeseries(out.simout), data.Time_ms_/1000);
rms_error_x = rms(data.x(2:end) - resampledSim.Data(2:end,1));
rms_error_y = rms(data.y(2:end) - resampledSim.Data(2:end,2));

error = norm([rms_error_x, rms_error_y]);
% error = rms_error_x;
end

