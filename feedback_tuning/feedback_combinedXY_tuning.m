fun = @batch_analysis_comb;

num_files = 10; % CHANGE THIS FOR BATCH TUNING
run_optimization = false; % false if you only want current parameters
print_ks = false; % true to easily copy parameters
plot_error_tracking = false;
plot_fullfeed_path = true;

PULLEY_RADIUS = readmatrix("../sys_id_data/pulley_radius.txt"); % Update each run from first line of raw csv

% current parameters, change as needed

kp_x = 1200;%0.05*o;%2.05114e+03/o;
ki_x = 1200;%0.001*o;%1.65375e+01/o;
kd_x = 10;%0.001*o;%1.47697e+01/o;
N_x = 100;

kp_y = 1200;%0.05*o;%7.98622e+03/o;
ki_y = 1200;%0.001*o;%1.08316e+01/o;
kd_y = 10;%0.004*o;%4.71861e+01/o;
N_y = 100;

%% No need to modify any code below 

cs = evalin('base','cs');
ds = evalin('base','ds');

if run_optimization
    options = optimset('PlotFcns',@optimplotfval);

    ks_xy0 = [kp_x  ki_x  kd_x  N_x kp_y  ki_y  kd_y  N_y  num_files];
    
    % continuous
    % xy_lower_bounds = [1e1, 1e0, 1e-1, 10, kp_y, ki_y, kd_y, N_y, num_files];
    % xy_upper_bounds = [1e5, 1e5, 1e3, 1e4, kp_y, ki_y, kd_y, N_y, num_files];
    % discrete
    xy_lower_bounds = [1e-3, 1e-2, 1e-2, 10, 1e-2, 1e-2, 1e-2, 10, num_files];
    xy_upper_bounds = [1e2, 1e2, 1e2, 1e3, 1e2, 1e2, 1e2, 1e2, num_files];
    [ks_xy,fval_xy,exitflag_xy,output_xy] = fminsearchbnd_comb(fun,ks_xy0,xy_lower_bounds,xy_upper_bounds,options);
    kp_x = ks_xy(1);
    ki_x = ks_xy(2);
    kd_x = ks_xy(3);
    N_x = floor(ks_xy(4));
    kp_y = ks_xy(5);
    ki_y = ks_xy(6);
    kd_y = ks_xy(7);
    N_y = floor(ks_xy(8));
end

if print_ks
    str_ks_x = [num2str(kp_x,'%.5e'), blanks(4), num2str(ki_x,'%.5e'), blanks(4), num2str(kd_x,'%.5e'), blanks(4), num2str(N_x)]
    str_ks_y = [num2str(kp_y,'%.5e'), blanks(4), num2str(ki_y,'%.5e'), blanks(4), num2str(kd_y,'%.5e'), blanks(4), num2str(N_y)]
end

%% Generate plots
generate_plots = plot_error_tracking || plot_fullfeed_path;
if generate_plots
    for i = 10:num_files

        file = strcat('feedforward_fit_data', num2str(i), '.mat');
        load(file, 'data');
        
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
        
        if plot_error_tracking
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
            
            f = figure('pos',[150,30,800,700]);
            
            subplot(2,2,1);
            hold on
            plot(data.Time_ms_ / 1000, data.Left_PWM*24, '-r', 'LineWidth',2);
            plot(data.Time_ms_ / 1000, resampledSim.Data(:,3), '-k','LineWidth',2);
            ylim([-1.1*24 1.1*24]);
            yticks('auto');
            yline([-24,24], '--', 'color', [0.6 0 0]);
            xlim([0,max(data.Time_ms_/1000)]);
            title('V1 Feedback Input');
            ylabel('PWM (-24V to 24V)');
            xlabel("Time (s)");
            
            subplot(2,2,2);
            hold on
            plot(data.Time_ms_ / 1000, data.Right_PWM*24, '-b', 'LineWidth',2);
            plot(data.Time_ms_ / 1000, resampledSim.Data(:,4), '-k','LineWidth',2);
            ylim([-1.1*24 1.1*24]);
            yticks('auto');
            yline([-24,24], '--', 'color', [0 0 0.6]);
            xlim([0,max(data.Time_ms_/1000)]);
            title('V2 Feedback Input');
            ylabel('PWM (-24V to 24V)');
            xlabel("Time (s)");
         
            subplot(2,2,3);
            hold on
            plot(data.Time_ms_ / 1000, data.x_error, 'ro','linest','none');
            plot(data.Time_ms_ / 1000,resampledSim.Data(:,1),'linewidth',2,'color',[0.6 0 0]);
            legend({'X Measured', 'X Sim'});
            xlabel("Time (s)");
            ylabel("Position (m)");
            title('X Feedback Error Tracking');
            xlim([0,max(data.Time_ms_)/1000]);
            grid on

            subplot(2,2,4);
            hold on
            plot(data.Time_ms_ / 1000, data.y_error, 'o','linest','none','color',[0 0.3 1]);
            plot(data.Time_ms_ / 1000,resampledSim.Data(:,2),'linewidth',2,'color',[0 0 0.6]);
            legend({'Y Error', 'Y Sim'});
            xlabel("Time (s)");
            ylabel("Position (m)");
            title('Y Feedback Error Tracking');
            xlim([0,max(data.Time_ms_)/1000]);
            grid on
        end


        if plot_fullfeed_path
            model = 'CoreXY_combined_fullfeed_model_XY';

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

            resampledSim = resample(timeseries(out.simout1), data.Time_ms_/1000);

            f = figure('pos',[0,30,1500,400]);

            subplot(1,2,1);
            hold on
            % X_pwm_input = sum([resampledSim.Data(:,3) data.X_PWM*24], 2);
            plot(data.Time_ms_ / 1000, resampledSim.Data(:,3), '-k','LineWidth',2);
            plot(data.Time_ms_ / 1000, data.Left_PWM*24, '-r', 'LineWidth',2);
            ylim([-1.1*24 1.1*24]);
            yticks('auto');
            yline([-24,24], '--', 'color', [0.6 0 0]);
            xlim([0,max(data.Time_ms_/1000)]);
            title('V1 Motor Input');
            ylabel('PWM (-24V to 24V)');
            xlabel('Time (s)');
            
            subplot(1,2,2);
            hold on
            % Y_pwm_input = sum([resampledSim.Data(:,4) data.Y_PWM*24], 2);
            plot(data.Time_ms_ / 1000, resampledSim.Data(:,4), '-k','LineWidth',2);
            plot(data.Time_ms_ / 1000, data.Right_PWM*24, '-b', 'LineWidth',2);
            ylim([-1.1*24 1.1*24]);
            yticks('auto');
            yline([-24,24], '--', 'color', [0 0 0.6]);
            xlim([0,max(data.Time_ms_/1000)]);
            title('V2 Motor Input');
            ylabel('PWM (-24V to 24V)');
            xlabel('Time (s)');

            
            xpos = resampledSim.Data(:,1);
            ypos = resampledSim.Data(:,2);

            g = figure('pos',[250,360,800,380]);
            hold on
            plot(data.x, data.y, 'ro','linest','none');
            plot(xpos,ypos,'linewidth',2,'color',[0.6 0 0]);
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