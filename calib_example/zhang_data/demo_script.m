% This demo script runs our calibration engine on the Zhengyou Zhang's data
% available at: http://www.research.microsoft.com/~zhang/Calib/
% This code loads the data from the format available on his web site, converts
% in my data format, runs the main calibration engine, displays the results and saves
% the results into a file called Calib_Results.mat

% (c) Jean-Yves Bouguet - Dec 24th, 1999


clear;

% Loads the 3D structures:
% All images have the same structure

load Model.txt;

X = Model(:,1:2:end);
Y = Model(:,2:2:end);

X = X(:)';
Y = Y(:)';

Np = length(X);

X_1 = [X;Y;zeros(1,Np)];

X_2 = X_1;
X_3 = X_1;
X_4 = X_1;
X_5 = X_1;


% Loads the Image coordinates

load data1.txt;
load data2.txt;
load data3.txt;
load data4.txt;
load data5.txt;

x = data1(:,1:2:end);
y = data1(:,2:2:end);
x = x(:)';
y = y(:)';
x_1 = [x;y];

x = data2(:,1:2:end);
y = data2(:,2:2:end);
x = x(:)';
y = y(:)';
x_2 = [x;y];

x = data3(:,1:2:end);
y = data3(:,2:2:end);
x = x(:)';
y = y(:)';
x_3 = [x;y];

x = data4(:,1:2:end);
y = data4(:,2:2:end);
x = x(:)';
y = y(:)';
x_4 = [x;y];

x = data5(:,1:2:end);
y = data5(:,2:2:end);
x = x(:)';
y = y(:)';
x_5 = [x;y];


% Loads the images:

calib_name = 'CalibIm';
format_image = 'tif';

ima_read_calib;


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