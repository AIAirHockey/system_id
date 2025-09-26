----- INSTRUCTIONS ---------
Run table on paths you would like to tune to. Save the experimental data for each path in a csv with the columns

x,y,dt,Left_PWM,Right_PWM

Where "Left" and "Right" are from the camera's perspective. Add the csv files to sys_id_data directory, then within
the feedforward_tuning directory, run feedforward_tuning.m (select desired settings at the top of the script) and 
select the csv files to be tuned to one at a time when prompted.

Output coefficients will be stored in variables 'as' and 'bs'.
