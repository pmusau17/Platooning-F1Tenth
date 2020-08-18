%% Load data
exp1 = load_experiment_data('data_25775.csv',1);
exp2 = load_experiment_data('data_27180.csv',1);
exp3 = load_experiment_data('data_27980.csv',1);
exp4 = load_experiment_data('data_29749.csv',1);
exp5 = load_experiment_data('data_28177.csv',1);
exp6 = load_experiment_data('racecar_walker_cclockwise_1.5.csv',1);
exp7 = load_experiment_data('racecar_walker_clockwise_1.0.csv',1);
exp8 = load_experiment_data('racecar_walker_clockwise_1.5.csv',1);

%% Visualize experiments
plot_race(exp1);
plot_race(exp2);
plot_race(exp3);
plot_race(exp4);
plot_race(exp5);
plot_race(exp6);
plot_race(exp7);
plot_race(exp8);

%% System Identification
% Create training and validating data
data_est = merge(exp1,exp2,exp3,exp5,exp7,exp6);
data_val = merge(exp4,exp8);
% Perpare estimation model
nx = 4; % order (number of states) of state-space model
opt = ssestOptions('InitialState','estimate','Display','on');
opt.Focus = 'prediction';
opt.SearchOptions.MaxIterations = 100;
opt.SearchMethod = 'lsqnonlin';
opt.SearchOptions.Advanced.UseParallel = true;
% Estimate model
sysid = ssest(data_est,nx,opt);
sysid2 = n4sid(data_est,nx,opt);
% Validate model
compare(data_val,sysid,sysid2,1);