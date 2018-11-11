addpath('TOOLBOX_calib');

format_image = 'jpg';
% Image Load
ima_read_calib;

% Extract corners
click_calib_no_read;

% Set the toolbox not to prompt the user (choose default values)

dont_ask = 1;

% Run the main calibration routine:

go_calib_optim;

% Shows the extrinsic parameters:

ext_calib;

% Reprojection on the original images:

reproject_calib;

% Saves the results into a file called Calib_Results.mat:

saving_calib;

% Set the toolbox to normal mode of operation again:

dont_ask =  0;

% Reproject on images
reproject_calib_no_read;
% Analyse error
analyse_error;
% Show Extrinsic
ext_calib;
% Switch to world-centered view
show_calib_results;
