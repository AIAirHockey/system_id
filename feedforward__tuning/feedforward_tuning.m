fun = @batch_analysis;

num_files = 10; % CHANGE THIS FOR BATCH TUNING
run_optimization = false; % false if you only want current parameters
print_as_bs = false; % true to easily copy parameters
print_cs_ds = true; % true to easily copy parameters
generate_plots = false;

PULLEY_RADIUS = 3.572890e-02; % Update each run from first line of raw csv

% current parameters, change as needed
% c0s = [5.2955e-06    8.449e-3    5.315e-2]; % original tuning from previous years
% d0s = [1.8625e-06    2.971e-3    5.315e-2];
% c0s = [3.5272e-05    1.01069e-2    6.65076e-2]; % pwm 75 tuning sept 24
% d0s = [2.45120e-05    5.02826e-3    8.27095e-2];
% c0s = [8.58174e-07    9.01747e-03    5.81510e-02]; % 12V batch tuning sept 29
% d0s = [5.17264e-06    4.19638e-03    7.98124e-02];
% c0s = [1.49793e-07    8.95124e-03    5.81427e-02]; % 12V 21-window batch oct 2
% d0s = [1.15775e-05    4.26005e-03    8.18171e-02]; % error (x,y) = (0.0087,0.0083)
% c0s = [9.08114e-06    9.45206e-03    6.29700e-02]; % 12V V2 oct 2
% d0s = [5.86763e-06    3.99030e-03    7.01893e-02]; % error (x,y) = (0.013,0.014)
c0s = [1.50741e-05    9.27855e-03    6.47259e-02]; % 12V supercap oct 10
d0s = [1.84756e-07    3.95581e-03    7.52809e-02]; % error (x,y) = (0.013,0.016)

%% No need to modify any code below 

cs=c0s;
ds=d0s;

if run_optimization
    options = optimset('PlotFcns',@optimplotfval);

    c0s = [c0s  0  num_files    PULLEY_RADIUS];
    d0s = [d0s  1  num_files    PULLEY_RADIUS];

    x_lower_bounds = [1.0e-8, 1.0e-4, 1.0e-4, 0, num_files, PULLEY_RADIUS];
    x_upper_bounds = [1.0e-3, 1.0e0, 1.0e0, 0, num_files, PULLEY_RADIUS];
    [cs,fval_x,exitflag_x,output_x] = fminsearchbnd(fun,c0s,x_lower_bounds,x_upper_bounds,options);
    
    y_lower_bounds = [1.0e-8, 1.0e-5, 1.0e-4, 1, num_files, PULLEY_RADIUS];
    y_upper_bounds = [1.0e-3, 1.0e-1, 1.0e0, 1, num_files, PULLEY_RADIUS];
    [ds,fval_y,exitflag_y,output_y] = fminsearchbnd(fun,d0s,y_lower_bounds,y_upper_bounds,options);
end

as = sum([cs(1:3); ds(1:3)])/2;
bs = diff([cs(1:3); ds(1:3)])/2;

if print_as_bs
    as_bs = [num2str(as(1),'%.3e'), ', ', num2str(as(2),'%.3e'), ', ', num2str(as(3),'%.3e'), ', ', num2str(bs(1),'%.3e'), ', ', num2str(bs(2),'%.3e'), ', ', num2str(bs(3),'%.3e')]
end

if print_cs_ds
    str_cs = [num2str(cs(1),'%.5e'), blanks(4), num2str(cs(2),'%.5e'), blanks(4), num2str(cs(3),'%.5e')]
    str_ds = [num2str(ds(1),'%.5e'), blanks(4), num2str(ds(2),'%.5e'), blanks(4), num2str(ds(3),'%.5e')]
end
%% Generate plots
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
      
            model = 'CoreXY_model_PWM_X';
            if j == 2
                model = 'CoreXY_model_PWM_Y';
            end
            simIn = Simulink.SimulationInput(model);
            simIn = setVariable(simIn,'data',data);
            simIn = setVariable(simIn, 'A',A);
            simIn = setVariable(simIn, 'B',B);
            simIn = setVariable(simIn, 'C',C);
            simIn = setVariable(simIn, 'D',D);
            out = sim(simIn);    
    
            resampledSim = resample(timeseries(out.simout), data.Time_ms_/1000);
    
            % f = figure('pos',[0,0,650,700]);
            % subplot(2,1,1);
            % hold on
            % 
            % if j == 1
            %     plot(data.Time_ms_ / 1000, data.X_PWM, '-k','LineWidth',2);
            %     ylim([-1.1 1.1]);
            %     title('X Motor Input');
            %     ylabel('PWM (-1 to 1)');
            % else
            %     f.Position = [650,0,650,700];
            %     plot(data.Time_ms_ / 1000, data.Y_PWM, '-k','LineWidth',2);
            %     ylim([-1.1 1.1]);
            %     title('Y Motor Input');
            %     ylabel('PWM (-1 to 1)');
            % end
            % 
            % subplot(2,1,2);
            % hold on
            % 
            % if j == 1
            %     plot(data.Time_ms_ / 1000, data.x, 'ro','linest','none');
            %     plot(out.simout.Time,out.simout.Data,'linewidth',2,'color',[0.6 0 0]);
            % else
            %     plot(data.Time_ms_ / 1000, data.y, 'o','linest','none','color',[0 0.3 1]);
            %     plot(out.simout.Time,out.simout.Data,'linewidth',2,'color',[0 0 0.6]);
            % end
            % 
            % title('Output');
            % if j == 1
            %     legend({'X Measured', 'X Sim'});
            % else
            %     legend({'Y Measured', 'Y Sim'});
            % end
            % xlabel("Time (s)");
            % ylabel("Position (m)");
            % grid on
            
            
            if j == 1
                xpos = resampledSim.Data;
            end
            if j == 2
                g = figure('pos',[250,320,800,380]);%[250,150,800,400]);
                hold on
                plot(data.x, data.y, 'ro','linest','none');
                plot(xpos,resampledSim.Data,'linewidth',2,'color',[0.6 0 0]);
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