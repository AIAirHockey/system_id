function [error] = CoreXY_feedback_analysis(x, filename)

kp = x(1);
ki = x(2);
kd = x(3);
flag = x(4);


A = evalin('base','A');
B = evalin('base','B');
C = evalin('base','C');
D = evalin('base','D');

% theta_rad_to_xy_cm = [1,-1;-1,-1]*PULLEY_RADIUS/2;

load(filename, 'data');

model = 'CoreXY_feedback_model_X';
if flag == 1
    model = 'CoreXY_feedback_model_Y';
end
simIn = Simulink.SimulationInput(model);
simIn = setVariable(simIn,'data',data);
simIn = setVariable(simIn, 'Kp', kp);
simIn = setVariable(simIn, 'Ki', ki);
simIn = setVariable(simIn, 'Kd', kd);
simIn = setVariable(simIn, 'A',A);
simIn = setVariable(simIn, 'B',B);
simIn = setVariable(simIn, 'C',C);
simIn = setVariable(simIn, 'D',D);
out = sim(simIn);

resampledSim = resample(timeseries(out.simout), data.Time_ms_/1000);

if flag == 0
    error = rms(data.x_error(2:end) - resampledSim.Data(2:end,1));
else
    error = rms(data.y_error(2:end) - resampledSim.Data(2:end,1));
end

end

