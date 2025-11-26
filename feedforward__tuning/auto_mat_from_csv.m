for i=1:254
    filename = strcat('feedforward_fit_data', num2str(i), '.mat');
    if ~isfile(fullfile("../sys_id_data/", filename))
        CoreXY_process_csv(filename, i);
    end
end