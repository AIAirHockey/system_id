function [error] = CoreXY_analysis_func(x, filename)

PULLEY_RADIUS = 3.5686666463209935e-2;

x1 = x(1);
x2 = x(2);
x3 = x(3);
flag = x(4);

s = tf('s');
Kff = 2/PULLEY_RADIUS*(x1*s^3+x2*s^2+x3*s);
inv_Kff = minreal(inv(Kff));
numer = inv_Kff.numerator{1,1};
denom = inv_Kff.denominator{1,1};

[A,B,C,D] = tf2ss(numer,denom);

% theta_rad_to_xy_cm = [1,-1;-1,-1]*PULLEY_RADIUS/2;

load(filename, 'data');

model = 'CoreXY_model_PWM_X';
if flag == 1
    model = 'CoreXY_model_PWM_Y';
end
simIn = Simulink.SimulationInput(model);
simIn = setVariable(simIn,'data',data);
% simIn = setVariable(simIn, 'theta_rad_to_xy_cm',theta_rad_to_xy_cm);
simIn = setVariable(simIn, 'A',A);
simIn = setVariable(simIn, 'B',B);
simIn = setVariable(simIn, 'C',C);
simIn = setVariable(simIn, 'D',D);
out = sim(simIn);
% out = sim(simIn,'SaveOutput','on','OutputSaveName','yout','SaveFormat','Dataset');

resampledSim = resample(timeseries(out.simout), data.Time_ms_/1000);

if flag == 1
    error = rms(data.y(2:end) - resampledSim.Data(2:end));
else
    error = rms(data.x(2:end) - resampledSim.Data(2:end));
end
% rms_error_x = rms(data.x(2:end) - resampledSim.Data(2:end,1));
% rms_error_y = rms(data.y(2:end) - resampledSim.Data(2:end,2));
% 
% error = norm([rms_error_x, rms_error_y]);
% error = rms_error_x;
end

