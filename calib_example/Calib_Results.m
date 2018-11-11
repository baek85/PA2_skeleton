% Intrinsic and Extrinsic Camera Parameters
%
% This script file can be directly executed under Matlab to recover the camera intrinsic and extrinsic parameters.
% IMPORTANT: This file contains neither the structure of the calibration objects nor the image coordinates of the calibration points.
%            All those complementary variables are saved in the complete matlab data file Calib_Results.mat.
% For more information regarding the calibration model visit http://www.vision.caltech.edu/bouguetj/calib_doc/


%-- Focal length:
fc = [ 14694.186490092957683 ; 8083.869849070483724 ];

%-- Principal point:
cc = [ 2669.150483887348400 ; 2084.233909630383096 ];

%-- Skew coefficient:
alpha_c = 0.000000000000000;

%-- Distortion coefficients:
kc = [ 3.586483841136618 ; 79.832020231260358 ; 0.307672246861108 ; 0.343498838244675 ; 0.000000000000000 ];

%-- Focal length uncertainty:
fc_error = [ 1101.720026300678910 ; 677.675035019715779 ];

%-- Principal point uncertainty:
cc_error = [ 186.733351021499089 ; 131.343092873559186 ];

%-- Skew coefficient uncertainty:
alpha_c_error = 0.000000000000000;

%-- Distortion coefficients uncertainty:
kc_error = [ 1.377593890288703 ; 39.391864537961553 ; 0.102764427992032 ; 0.084519453483225 ; 0.000000000000000 ];

%-- Image size:
nx = 4032;
ny = 3024;


%-- Various other variables (may be ignored if you do not use the Matlab Calibration Toolbox):
%-- Those variables are used to control which intrinsic parameters should be optimized

n_ima = 14;						% Number of calibration images
est_fc = [ 1 ; 1 ];					% Estimation indicator of the two focal variables
est_aspect_ratio = 1;				% Estimation indicator of the aspect ratio fc(2)/fc(1)
center_optim = 1;					% Estimation indicator of the principal point
est_alpha = 0;						% Estimation indicator of the skew coefficient
est_dist = [ 1 ; 1 ; 1 ; 1 ; 0 ];	% Estimation indicator of the distortion coefficients


%-- Extrinsic parameters:
%-- The rotation (omc_kk) and the translation (Tc_kk) vectors for every calibration image and their uncertainties

%-- Image #1:
omc_1 = [ 1.784808e+00 ; 1.746813e+00 ; -5.414176e-01 ];
Tc_1  = [ -1.186669e+03 ; -1.412768e+03 ; 1.052876e+04 ];
omc_error_1 = [ 3.801460e-02 ; 3.506630e-02 ; 7.833333e-02 ];
Tc_error_1  = [ 1.442278e+02 ; 1.883710e+02 ; 5.972463e+02 ];

%-- Image #2:
omc_2 = [ NaN ; NaN ; NaN ];
Tc_2  = [ NaN ; NaN ; NaN ];
omc_error_2 = [ NaN ; NaN ; NaN ];
Tc_error_2  = [ NaN ; NaN ; NaN ];

%-- Image #3:
omc_3 = [ NaN ; NaN ; NaN ];
Tc_3  = [ NaN ; NaN ; NaN ];
omc_error_3 = [ NaN ; NaN ; NaN ];
Tc_error_3  = [ NaN ; NaN ; NaN ];

%-- Image #4:
omc_4 = [ 1.771726e+00 ; 1.675093e+00 ; -7.364583e-01 ];
Tc_4  = [ -8.013926e+02 ; -2.286863e+03 ; 1.255064e+04 ];
omc_error_4 = [ 4.101966e-02 ; 2.587309e-02 ; 4.909412e-02 ];
Tc_error_4  = [ 1.783383e+02 ; 2.184544e+02 ; 7.364841e+02 ];

%-- Image #5:
omc_5 = [ 1.944785e+00 ; 1.627489e+00 ; -9.718094e-01 ];
Tc_5  = [ -1.782228e+03 ; -2.179309e+03 ; 1.212523e+04 ];
omc_error_5 = [ 3.734171e-02 ; 3.287522e-02 ; 5.347472e-02 ];
Tc_error_5  = [ 1.764538e+02 ; 2.319601e+02 ; 6.984709e+02 ];

%-- Image #6:
omc_6 = [ NaN ; NaN ; NaN ];
Tc_6  = [ NaN ; NaN ; NaN ];
omc_error_6 = [ NaN ; NaN ; NaN ];
Tc_error_6  = [ NaN ; NaN ; NaN ];

%-- Image #7:
omc_7 = [ 1.667851e+00 ; 1.737208e+00 ; -5.395675e-01 ];
Tc_7  = [ -1.317905e+03 ; -8.013381e+02 ; 1.359302e+04 ];
omc_error_7 = [ 3.448897e-02 ; 3.318840e-02 ; 7.837304e-02 ];
Tc_error_7  = [ 1.741405e+02 ; 2.345594e+02 ; 7.895927e+02 ];

%-- Image #8:
omc_8 = [ -5.504986e-01 ; -1.818642e+00 ; -1.337669e-01 ];
Tc_8  = [ -5.471256e+02 ; -8.763492e+02 ; 7.962030e+03 ];
omc_error_8 = [ 1.637744e-02 ; 1.482690e-02 ; 3.876418e-02 ];
Tc_error_8  = [ 1.017888e+02 ; 1.292134e+02 ; 5.647301e+02 ];

%-- Image #9:
omc_9 = [ -9.081427e-01 ; -2.070001e+00 ; 4.754523e-01 ];
Tc_9  = [ -5.879837e+02 ; -1.487946e+03 ; 1.004520e+04 ];
omc_error_9 = [ 1.837589e-02 ; 3.526117e-02 ; 5.414189e-02 ];
Tc_error_9  = [ 1.373757e+02 ; 1.730682e+02 ; 6.842888e+02 ];

%-- Image #10:
omc_10 = [ -2.371447e+00 ; -8.261787e-01 ; 5.194742e-01 ];
Tc_10  = [ -1.463140e+03 ; -1.549415e+03 ; 1.333592e+04 ];
omc_error_10 = [ 5.230781e-02 ; 1.604371e-02 ; 9.681026e-02 ];
Tc_error_10  = [ 1.611493e+02 ; 2.217046e+02 ; 7.999908e+02 ];

%-- Image #11:
omc_11 = [ -1.610235e+00 ; -1.492644e+00 ; 4.234532e-01 ];
Tc_11  = [ -9.327765e+02 ; -1.530135e+03 ; 1.123003e+04 ];
omc_error_11 = [ 2.908251e-02 ; 1.617965e-02 ; 5.668645e-02 ];
Tc_error_11  = [ 1.442753e+02 ; 1.803247e+02 ; 7.082973e+02 ];

%-- Image #12:
omc_12 = [ -1.584524e+00 ; -1.678280e+00 ; 8.027456e-01 ];
Tc_12  = [ -1.160531e+03 ; -9.865740e+02 ; 1.188753e+04 ];
omc_error_12 = [ 2.757813e-02 ; 2.739512e-02 ; 5.344422e-02 ];
Tc_error_12  = [ 1.550984e+02 ; 2.097307e+02 ; 7.716616e+02 ];

%-- Image #13:
omc_13 = [ 2.309538e+00 ; 7.465168e-01 ; 1.598261e+00 ];
Tc_13  = [ -1.460726e+03 ; 1.607028e+02 ; 1.023738e+04 ];
omc_error_13 = [ 3.986807e-02 ; 2.840266e-02 ; 3.416717e-02 ];
Tc_error_13  = [ 1.331868e+02 ; 1.879459e+02 ; 7.977239e+02 ];

%-- Image #14:
omc_14 = [ -1.582150e+00 ; -8.632135e-01 ; 8.510165e-01 ];
Tc_14  = [ -4.796926e+02 ; -6.716047e+02 ; 8.260759e+03 ];
omc_error_14 = [ 1.852254e-02 ; 1.544413e-02 ; 2.776761e-02 ];
Tc_error_14  = [ 1.042077e+02 ; 1.345076e+02 ; 4.590218e+02 ];

