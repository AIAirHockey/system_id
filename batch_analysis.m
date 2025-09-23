function [rms_error] = batch_analysis(x)

n = 4;
errors = zeros(1,n);
for i=1:n
    filename = strcat('fit_data', num2str(i), '.mat');
    CoreXY_process_csv(filename);
    errors(i) = CoreXY_analysis_func(x, filename);
end
    rms_error = rms(errors);
end

