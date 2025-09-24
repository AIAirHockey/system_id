function [rms_error] = batch_analysis(x)

n = 1;
errors = zeros(1,n);
for i=1:n
    filename = strcat('feedforward_fit_data', num2str(i), '.mat');
    if ~isfile(fullfile("sys_id_data/", filename))
        CoreXY_process_csv(filename);
    end
    errors(i) = CoreXY_analysis_func(x, filename);
end
    rms_error = rms(errors);
end

