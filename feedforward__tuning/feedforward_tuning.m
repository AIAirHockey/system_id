fun = @batch_analysis;

num_files = 1; % CHANGE THIS FOR BATCH TUNING
run_optimization = false; % false if you only want current parameters
print_as_bs = false; % true to easily copy parameters
generate_plots = true;

% current parameters, change as needed
c0s = [3.5272e-05    1.01069e-2    6.65076e-2]; % pwm 75 tuning sept 24
d0s = [2.45120e-05    5.02826e-3    8.27095e-2]; % pwm 75 tuning sept 24

%% No need to modify any code below 
PULLEY_RADIUS = 3.5686666463209935e-2;

cs=c0s;
ds=d0s;

if run_optimization
    options = optimset('PlotFcns',@optimplotfval);

    c0s = [c0s  0  num_files];
    d0s = [d0s  1  num_files];

    x_lower_bounds = [1.0e-8, 1.0e-4, 1.0e-4, 0, num_files];
    x_upper_bounds = [1.0e-3, 1.0e0, 1.0e0, 0, num_files];
    [cs,fval_x,exitflag_x,output_x] = fminsearchbnd(fun,c0s,x_lower_bounds,x_upper_bounds,options);
    
    y_lower_bounds = [1.0e-8, 1.0e-5, 1.0e-4, 1, num_files];
    y_upper_bounds = [1.0e-3, 1.0e-1, 1.0e0, 1, num_files];
    [ds,fval_y,exitflag_y,output_y] = fminsearchbnd(fun,d0s,y_lower_bounds,y_upper_bounds,options);
end

as = sum([cs(1:3); ds(1:3)])/2;
bs = diff([cs(1:3); ds(1:3)])/2;

if print_as_bs
    as_bs = strcat(num2str(as(1),'%.3e'), ', ', num2str(as(2),'%.3e'), ', ', num2str(as(3),'%.3e'), ', ', num2str(bs(1),'%.3e'), ', ', num2str(bs(2),'%.3e'), ', ', num2str(bs(3),'%.3e'))
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
    
            f = figure('pos',[0,0,650,700]);
            subplot(2,1,1);
            hold on
            
            if j == 1
                plot(data.Time_ms_ / 1000, data.X_PWM, '-k','LineWidth',2);
                ylim([-1.1 1.1]);
                title('X Motor Input');
                ylabel('PWM (-1 to 1)');
            else
                f.Position = [650,0,650,700];
                plot(data.Time_ms_ / 1000, data.Y_PWM, '-k','LineWidth',2);
                ylim([-1.1 1.1]);
                title('Y Motor Input');
                ylabel('PWM (-1 to 1)');
            end
    
            subplot(2,1,2);
            hold on
            
            if j == 1
                plot(data.Time_ms_ / 1000, data.x, 'ro','linest','none');
                plot(out.simout.Time,out.simout.Data,'linewidth',2,'color',[0.6 0 0]);
            else
                plot(data.Time_ms_ / 1000, data.y, 'o','linest','none','color',[0 0.3 1]);
                plot(out.simout.Time,out.simout.Data,'linewidth',2,'color',[0 0 0.6]);
            end
    
            title('Output');
            if j == 1
                legend({'X Measured', 'X Sim'});
            else
                legend({'Y Measured', 'Y Sim'});
            end
            xlabel("Time (s)");
            ylabel("Position (m)");
            grid on
            
            
            if j == 1
                xpos = resampledSim.Data;
            end
            if j == 2
                g = figure('pos',[250,150,800,400]);
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