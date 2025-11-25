function [rms_error] = batch_analysis_comb(x)

num_files = x(9);
errors = zeros(1,num_files);
for i=1:num_files
    filename = strcat('feedforward_fit_data', num2str(i), '.mat');
    if ~isfile(fullfile("../sys_id_data/", filename))
        CoreXY_process_csv(filename);
    end
    errors(i) = CoreXY_combined_feedback_analysis(x, filename);
end
    rms_error = rms(errors);
end

