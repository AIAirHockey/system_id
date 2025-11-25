function [error] = CoreXY_combined_feedback_analysis(x, filename)

kp_x = x(1);
ki_x = x(2);
kd_x = x(3);
N_x = x(4);
kp_y = x(5);
ki_y = x(6);
kd_y = x(7);
N_y = x(8);

%% Get SS systems
PULLEY_RADIUS = readmatrix("../sys_id_data/pulley_radius.txt");
cs = evalin('base','cs');
ds = evalin('base','ds');

x1 = cs(1);
x2 = cs(2);
x3 = cs(3);

s = tf('s');
Kff_x = 2/PULLEY_RADIUS*(x1*s^3+x2*s^2+x3*s);
inv_Kff_x = minreal(inv(Kff_x));
numer_x = inv_Kff_x.numerator{1,1};
denom_x = inv_Kff_x.denominator{1,1};

[A_x,B_x,C_x,D_x] = tf2ss(numer_x,denom_x);

y1 = ds(1);
y2 = ds(2);
y3 = ds(3);

Kff_y = 2/PULLEY_RADIUS*(y1*s^3+y2*s^2+y3*s);
inv_Kff_y = minreal(inv(Kff_y));
numer_y = inv_Kff_y.numerator{1,1};
denom_y = inv_Kff_y.denominator{1,1};

[A_y,B_y,C_y,D_y] = tf2ss(numer_y,denom_y);

% theta_rad_to_xy_cm = [1,-1;-1,-1]*PULLEY_RADIUS/2;

%% Sim and get errors
load(filename, 'data');

model = 'CoreXY_combined_feedback_model_XY';
simIn = Simulink.SimulationInput(model);
simIn = setVariable(simIn,'data',data);
simIn = setVariable(simIn, 'Kp_X', kp_x);
simIn = setVariable(simIn, 'Ki_X', ki_x);
simIn = setVariable(simIn, 'Kd_X', kd_x);
simIn = setVariable(simIn, 'N_X', N_x);
simIn = setVariable(simIn, 'Kp_Y', kp_y);
simIn = setVariable(simIn, 'Ki_Y', ki_y);
simIn = setVariable(simIn, 'Kd_Y', kd_y);
simIn = setVariable(simIn, 'N_Y', N_y);
simIn = setVariable(simIn, 'A_X',A_x);
simIn = setVariable(simIn, 'B_X',B_x);
simIn = setVariable(simIn, 'C_X',C_x);
simIn = setVariable(simIn, 'D_X',D_x);
simIn = setVariable(simIn, 'A_Y',A_y);
simIn = setVariable(simIn, 'B_Y',B_y);
simIn = setVariable(simIn, 'C_Y',C_y);
simIn = setVariable(simIn, 'D_Y',D_y);
out = sim(simIn);

resampledSim = resample(timeseries(out.simout), data.Time_ms_/1000);

%% error calc
error = rms(data.x_error(2:end) - resampledSim.Data(2:end,1));
error = error + rms(data.y_error(2:end) - resampledSim.Data(2:end,2));

V1 = resampledSim.Data(:,3);
V2 = resampledSim.Data(:,4);

idx1 = find(abs(V1)>24);
idx2 = find(abs(V2)>24);

if ~isempty(idx1)
error = error + sum(abs(V1(idx1))) - 24*length(idx1);
end
if ~isempty(idx2)
error = error + sum(abs(V2(idx2))) - 24*length(idx2);
end

end

