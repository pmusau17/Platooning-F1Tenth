% validate the sysid model returned from the grey-box system identification
% specify the parameters obtained from the sys-id in the file
% simulate_bicycle_euler.m

mse = validate_experiment('csv/data_15870_0.csv');
mse1 = validate_experiment('csv/data_15870_2.csv');
mse2 = validate_experiment('csv/data_15870_3.csv');
mse3 = validate_experiment('csv/data_15870_5.csv');
mse4 = validate_experiment('csv/data_15870_6.csv');
mse5 = validate_experiment('csv/data_15870_8.csv');
mse6 = validate_experiment('csv/data_15870_9.csv');
mses = [mse,mse1,mse2,mse3,mse4,mse5,mse6];
avg_mse = mean(mses);