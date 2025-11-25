fun = @batch_analysis;

num_files = 10; % CHANGE THIS FOR BATCH TUNING
run_optimization = false; % false if you only want current parameters
print_ks = true; % true to easily copy parameters
plot_error_tracking = false;
plot_feedback_path = true;

PULLEY_RADIUS = readmatrix("../../sys_id_data/pulley_radius.txt"); % Update each run from first line of raw csv

% current parameters, change as needed
o = 8;
kp_y = 1.55525e+03/o;%6.48646e+04/o;
ki_y = 8.34134e+03/o;%9.72335e+04/o;
kd_y = 1.26202e+02/o;%9.99962e+02/o;

kp_x = 1.55525e+03/o;
ki_x = 8.34134e+03/o;
kd_x = 1.26202e+02/o;

%% No need to modify any code below 

cs = evalin('base','cs');
ds = evalin('base','ds');

if run_optimization
    options = optimset('PlotFcns',@optimplotfval);

    ks_x0 = [kp_x  ki_x  kd_x  0  num_files];
    ks_y0 = [kp_y  ki_y  kd_y  1  num_files];

    x_lower_bounds = [1e1, 1e0, 1e-1, 0, num_files];
    x_upper_bounds = [1e5, 1e5, 1e3, 0, num_files];
    [ks_x,fval_x,exitflag_x,output_x] = fminsearchbnd(fun,ks_x0,x_lower_bounds,x_upper_bounds,options);
    kp_x = ks_x(1);
    ki_x = ks_x(2);
    kd_x = ks_x(3);

    % y_lower_bounds = [1e1, 1e0, 1e-1, 1, num_files];
    % y_upper_bounds = [1e5, 1e5, 1e3, 1, num_files];
    % [ks_y,fval_y,exitflag_y,output_y] = fminsearchbnd(fun,ks_y0,y_lower_bounds,y_upper_bounds,options);
    % kp_y = ks_y(1);
    % ki_y = ks_y(2);
    % kd_y = ks_y(3);
end

if print_ks
    str_ks_x = [num2str(kp_x,'%.5e'), blanks(4), num2str(ki_x,'%.5e'), blanks(4), num2str(kd_x,'%.5e')]
    str_ks_y = [num2str(kp_y,'%.5e'), blanks(4), num2str(ki_y,'%.5e'), blanks(4), num2str(kd_y,'%.5e')]
end

%% Generate plots
generate_plots = plot_error_tracking || plot_feedback_path;
if generate_plots
    for i = 1:num_files
        for j = 1:2
            if j == 1
                x = cs;
            else
                x = ds;
            end
    
            file = strcat('feedforward_fit_data', num2str(i), '.mat');
            load(file, 'data');
            
            x1 = x(1);
            x2 = x(2);
            x3 = x(3);
    
            s = tf('s');
            Kff = 2/PULLEY_RADIUS*(x1*s^3+x2*s^2+x3*s);
            inv_Kff = minreal(inv(Kff));
            numer = inv_Kff.numerator{1,1};
            denom = inv_Kff.denominator{1,1};
            
            [A,B,C,D] = tf2ss(numer,denom);
            
            if plot_error_tracking
                if j == 1
                    model = 'CoreXY_feedback_model_X';
                else
                    model = 'CoreXY_feedback_model_Y';
                end

                simIn = Simulink.SimulationInput(model);
                simIn = setVariable(simIn,'data',data);
                if j == 1
                    simIn = setVariable(simIn, 'Kp', kp_x);
                    simIn = setVariable(simIn, 'Ki', ki_x);
                    simIn = setVariable(simIn, 'Kd', kd_x);
                else
                    simIn = setVariable(simIn, 'Kp', kp_y);
                    simIn = setVariable(simIn, 'Ki', ki_y);
                    simIn = setVariable(simIn, 'Kd', kd_y);
                end
                simIn = setVariable(simIn, 'A',A);
                simIn = setVariable(simIn, 'B',B);
                simIn = setVariable(simIn, 'C',C);
                simIn = setVariable(simIn, 'D',D);
                out = sim(simIn);

                resampledSim = resample(timeseries(out.simout), data.Time_ms_/1000);
                
                f = figure('pos',[0,0,650,700]);
                subplot(2,1,1);
                hold on
                if j == 1
                    plot(data.Time_ms_ / 1000, resampledSim.Data(:,2)/24, '-k','LineWidth',2);
                    ylim([-1.1 1.1]);
                    xlim([0,max(data.Time_ms_/1000)]);
                    title('X Motor Input');
                    ylabel('PWM (-1 to 1)');
                else
                    f.Position = [650,0,650,700];
                    plot(data.Time_ms_ / 1000, resampledSim.Data(:,2)/24, '-k','LineWidth',2);
                    ylim([-1.1 1.1]);
                    xlim([0,max(data.Time_ms_/1000)]);
                    title('Y Motor Input');
                    ylabel('PWM (-1 to 1)');
                end
                
                subplot(2,1,2);
                hold on
    
                if j == 1
                    plot(data.Time_ms_ / 1000, data.x_error, 'ro','linest','none');
                    plot(data.Time_ms_ / 1000,resampledSim.Data(:,1),'linewidth',2,'color',[0.6 0 0]);
                else
                    plot(data.Time_ms_ / 1000, data.y_error, 'o','linest','none','color',[0 0.3 1]);
                    plot(data.Time_ms_ / 1000,resampledSim.Data(:,1),'linewidth',2,'color',[0 0 0.6]);
                end
    
                title('Output');
                if j == 1
                    legend({'X Measured', 'X Sim'});
                else
                    legend({'Y Error', 'Y Sim'});
                end
                xlabel("Time (s)");
                ylabel("Position (m)");
                xlim([0,max(data.Time_ms_)/1000]);
                grid on

            end
            if plot_feedback_path
                if j == 1
                    model = 'CoreXY_fullfeed_model_PWM_X';
                else
                    model = 'CoreXY_fullfeed_model_PWM_Y';
                end

                simIn = Simulink.SimulationInput(model);
                simIn = setVariable(simIn,'data',data);
                if j == 1
                    simIn = setVariable(simIn, 'Kp', kp_x);
                    simIn = setVariable(simIn, 'Ki', ki_x);
                    simIn = setVariable(simIn, 'Kd', kd_x);
                else
                    simIn = setVariable(simIn, 'Kp', kp_y);
                    simIn = setVariable(simIn, 'Ki', ki_y);
                    simIn = setVariable(simIn, 'Kd', kd_y);
                end
                simIn = setVariable(simIn, 'A',A);
                simIn = setVariable(simIn, 'B',B);
                simIn = setVariable(simIn, 'C',C);
                simIn = setVariable(simIn, 'D',D);
                out = sim(simIn); 

                resampledSim = resample(timeseries(out.simout2), data.Time_ms_/1000);

                f = figure('pos',[0,0,650,400]);
                hold on
    
                if j == 1
                    X_pwm_input = resampledSim.Data(:,2)/24 + data.X_PWM;
                    plot(data.Time_ms_ / 1000, X_pwm_input, '-k','LineWidth',2);
                    ylim([-1.1 1.1]);
                    xlim([0,max(data.Time_ms_/1000)]);
                    title('X Motor Input');
                    ylabel('PWM (-1 to 1)');
                else
                    Y_pwm_input = resampledSim.Data(:,2)/24 + data.Y_PWM;
                    f.Position = [650,0,650,400];
                    plot(data.Time_ms_ / 1000, Y_pwm_input, '-k','LineWidth',2);
                    ylim([-1.1 1.1]);
                    xlim([0,max(data.Time_ms_/1000)]);
                    title('Y Motor Input');
                    ylabel('PWM (-1 to 1)');
                end

                if j == 1
                    xpos = resampledSim.Data(:,1);
                end
                if j == 2
                    g = figure('pos',[250,320,800,380]);%[250,150,800,400]);
                    hold on
                    plot(data.x, data.y, 'ro','linest','none');
                    plot(xpos,resampledSim.Data(:,1),'linewidth',2,'color',[0.6 0 0]);
                    title('Path in XY Space')
                    legend({'Real Path', 'Sim Path'});
                    xlim([0 2]);
                    ylim([0 1]);
                    xlabel("X Position (m)");
                    ylabel("Y Position (m)");
                    grid on
                end
            end
        end
    end
end