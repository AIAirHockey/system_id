fun = @batch_analysis;

num_files = 158; % CHANGE THIS FOR BATCH TUNING
run_optimization = false; % false if you only want current parameters
print_as_bs = false; % true to easily copy parameters
print_cs_ds = false; % true to easily copy parameters
generate_plots = true;

PULLEY_RADIUS = readmatrix("../sys_id_data/pulley_radius.txt"); % Update each run from first line of raw csv

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
% c0s = [1.50741e-05    9.27855e-03    6.47259e-02]; % 12V supercap oct 10
% d0s = [1.84756e-07    3.95581e-03    7.52809e-02]; % error (x,y) = (0.013,0.016)
% c0s = [2.76926e-05    9.42916e-03    6.50252e-02]; % 12V supercap [91,35] window oct 12
% d0s = [2.12640e-07    3.98341e-03    7.56456e-02]; % error (x,y) = (0.012,0.017)
% c0s = [4.34440e-05    9.32182e-03    6.33829e-02]; % 5V-19.6V delayed nov 4
% d0s = [5.29217e-08    3.61067e-03    7.41763e-02]; % error (x,y) = (0.014,0.021)
% c0s = [4.72150e-05    1.04130e-02    6.59989e-02]; % 5V-19.6V 66-batch nov 4
% d0s = [7.28582e-06    4.73660e-03    7.33742e-02]; % error (x,y) = (0.012,0.024)
% c0s = [6.14975e-05    1.37772e-02    8.14952e-02]; % 171 files 0.8VmaxFF Vtotal nov 6
% d0s = [9.80875e-07    6.17135e-03    1.01995e-01]; % error (x,y) = (0.034,0.029)
c0s = [7.35474e-05    1.39117e-02    8.31498e-02]; % 158 files w/o low voltage nov 7
d0s = [1.03803e-08    5.95144e-03    9.21948e-02]; % error (x,y) = (0.035,0.023)

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
    disp(fval_x)
    disp(output_x.iterations)

    y_lower_bounds = [1.0e-8, 1.0e-5, 1.0e-4, 1, num_files, PULLEY_RADIUS];
    y_upper_bounds = [1.0e-3, 1.0e-1, 1.0e0, 1, num_files, PULLEY_RADIUS];
    [ds,fval_y,exitflag_y,output_y] = fminsearchbnd(fun,d0s,y_lower_bounds,y_upper_bounds,options);
    disp(fval_y)
    disp(output_y.iterations)
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

xpos = [];
ypos = [];
index = [];
% time = [];
if generate_plots
    for i = 1:num_files
        file = strcat('feedforward_fit_data', num2str(i), '.mat');
        load(file, 'data');
        for j = 1:2
            if j == 1
                x = cs;
            else
                x = ds;
            end
            
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

            % w = 21;
            % if j == 1
            %     x_error = resampledSim.Data - data.x;
            %     x_error = movmean(x_error, w);
            %     data = addvars(data, x_error);
            % else
            %     y_error = resampledSim.Data - data.y;
            %     y_error = movmean(y_error, w);
            %     data = addvars(data, y_error);
            % end
            % save(file, 'data', '-append');

               
            %
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
            %
            
            if j == 1
                xpos = [xpos; resampledSim.Data];
                index = [index; data.index(1:length(resampledSim.Data))];
                % if isempty(time)
                %     time = resampledSim.Time;
                % else
                %     start_next = time(end);
                %     time = [time; resampledSim.Time + start_next + data.dt(1)/1000];
                % end
            end
            if j == 2
                ypos = [ypos; resampledSim.Data];
                % g = figure('pos',[250,320,800,380]);%[250,150,800,400]);
                % hold on
                % plot(data.x, data.y, 'ro','linest','none');
                % plot(xpos,resampledSim.Data,'linewidth',2,'color',[0.6 0 0]);
                % title('Path in XY Space')
                % legend({'Real Path', 'Sim Path'});
                % xlim([0 2]);
                % ylim([0 1]);
                % xlabel("X Position (m)");
                % ylabel("Y Position (m)");
                % grid on
            end
        end
        % waitfor(g);
    end
    T = table(index, xpos, ypos, 'VariableNames', {'index', 'x','y'});
    writetable(T, '../sys_id_data/feedforward_sim_data_NN_NoLow.csv');
end