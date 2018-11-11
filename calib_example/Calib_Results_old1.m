% Intrinsic and Extrinsic Camera Parameters
%
% This script file can be directly executed under Matlab to recover the camera intrinsic and extrinsic parameters.
% IMPORTANT: This file contains neither the structure of the calibration objects nor the image coordinates of the calibration points.
%            All those complementary variables are saved in the complete matlab data file Calib_Results.mat.
% For more information regarding the calibration model visit http://www.vision.caltech.edu/bouguetj/calib_doc/


%-- Focal length:
fc = [ 659.032459505586417 ; 659.641032682326454 ];

%-- Principal point:
cc = [ 319.500000000000000 ; 239.500000000000000 ];

%-- Skew coefficient:
alpha_c = 0.000000000000000;

%-- Distortion coefficients:
kc = [ -0.261658854835965 ; 0.170023458621607 ; -0.000492194188600 ; 0.002110619127765 ; 0.000000000000000 ];

%-- Focal length uncertainty:
fc_error = [ 0.567639667819814 ; 0.635703621749122 ];

%-- Principal point uncertainty:
cc_error = [ 0.000000000000000 ; 0.000000000000000 ];

%-- Skew coefficient uncertainty:
alpha_c_error = 0.000000000000000;

%-- Distortion coefficients uncertainty:
kc_error = [ 0.005746894267630 ; 0.027833616874181 ; 0.000220484426817 ; 0.000201805348367 ; 0.000000000000000 ];

%-- Image size:
nx = 640;
ny = 480;


%-- Various other variables (may be ignored if you do not use the Matlab Calibration Toolbox):
%-- Those variables are used to control which intrinsic parameters should be optimized

n_ima = 20;						% Number of calibration images
est_fc = [ 1 ; 1 ];					% Estimation indicator of the two focal variables
est_aspect_ratio = 1;				% Estimation indicator of the aspect ratio fc(2)/fc(1)
center_optim = 0;					% Estimation indicator of the principal point
est_alpha = 0;						% Estimation indicator of the skew coefficient
est_dist = [ 1 ; 1 ; 1 ; 1 ; 0 ];	% Estimation indicator of the distortion coefficients


%-- Extrinsic parameters:
%-- The rotation (omc_kk) and the translation (Tc_kk) vectors for every calibration image and their uncertainties

%-- Image #1:
omc_1 = [ 1.652864e+00 ; 1.632114e+00 ; -6.506547e-01 ];
Tc_1  = [ -1.989330e+02 ; -7.972846e+01 ; 8.505561e+02 ];
omc_error_1 = [ 5.705439e-04 ; 6.042103e-04 ; 9.293639e-04 ];
Tc_error_1  = [ 7.196868e-02 ; 1.186646e-01 ; 6.965573e-01 ];

%-- Image #2:
omc_2 = [ 1.842416e+00 ; 1.881798e+00 ; -3.773999e-01 ];
Tc_2  = [ -1.740389e+02 ; -1.557038e+02 ; 7.560456e+02 ];
omc_error_2 = [ 7.217600e-04 ; 7.636928e-04 ; 1.432390e-03 ];
Tc_error_2  = [ 6.575452e-02 ; 1.089377e-01 ; 6.858778e-01 ];

%-- Image #3:
omc_3 = [ 1.737238e+00 ; 2.058071e+00 ; -4.864118e-01 ];
Tc_3  = [ -1.447408e+02 ; -1.709043e+02 ; 7.746890e+02 ];
omc_error_3 = [ 7.116150e-04 ; 8.170602e-04 ; 1.498111e-03 ];
Tc_error_3  = [ 7.321771e-02 ; 1.178800e-01 ; 6.701552e-01 ];

%-- Image #4:
omc_4 = [ 1.830275e+00 ; 2.099098e+00 ; -1.078293e+00 ];
Tc_4  = [ -8.407982e+01 ; -1.512319e+02 ; 7.793125e+02 ];
omc_error_4 = [ 6.828161e-04 ; 6.335930e-04 ; 9.204238e-04 ];
Tc_error_4  = [ 9.326274e-02 ; 1.129383e-01 ; 5.602648e-01 ];

%-- Image #5:
omc_5 = [ 1.074708e+00 ; 1.898738e+00 ; -2.433248e-01 ];
Tc_5  = [ -1.109595e+02 ; -2.255130e+02 ; 7.368780e+02 ];
omc_error_5 = [ 3.763823e-04 ; 4.744798e-04 ; 8.178143e-04 ];
Tc_error_5  = [ 9.083607e-02 ; 8.905791e-02 ; 6.444386e-01 ];

%-- Image #6:
omc_6 = [ -1.701148e+00 ; -1.949181e+00 ; -8.134943e-01 ];
Tc_6  = [ -1.600431e+02 ; -7.752143e+01 ; 4.425600e+02 ];
omc_error_6 = [ 5.280093e-04 ; 5.726650e-04 ; 8.175947e-04 ];
Tc_error_6  = [ 7.430095e-02 ; 8.439190e-02 ; 5.435304e-01 ];

%-- Image #7:
omc_7 = [ 1.968473e+00 ; 1.922655e+00 ; 1.323980e+00 ];
Tc_7  = [ -9.408170e+01 ; -7.566217e+01 ; 4.397562e+02 ];
omc_error_7 = [ 5.426729e-04 ; 5.834343e-04 ; 7.394207e-04 ];
Tc_error_7  = [ 9.712429e-02 ; 8.555077e-02 ; 6.176360e-01 ];

%-- Image #8:
omc_8 = [ 1.935399e+00 ; 1.815519e+00 ; 1.339024e+00 ];
Tc_8  = [ -1.821928e+02 ; -1.012630e+02 ; 4.603529e+02 ];
omc_error_8 = [ 6.073709e-04 ; 6.622893e-04 ; 8.412808e-04 ];
Tc_error_8  = [ 1.520269e-01 ; 9.859040e-02 ; 6.901576e-01 ];

%-- Image #9:
omc_9 = [ -1.375919e+00 ; -2.000625e+00 ; 3.112996e-01 ];
Tc_9  = [ -2.026327e+01 ; -2.217132e+02 ; 7.305879e+02 ];
omc_error_9 = [ 5.956147e-04 ; 8.267065e-04 ; 1.169205e-03 ];
Tc_error_9  = [ 1.091439e-01 ; 5.838579e-02 ; 6.933733e-01 ];

%-- Image #10:
omc_10 = [ -1.525564e+00 ; -2.104524e+00 ; 1.745032e-01 ];
Tc_10  = [ -5.170354e+01 ; -2.961415e+02 ; 8.616965e+02 ];
omc_error_10 = [ 9.721755e-04 ; 1.206003e-03 ; 1.918082e-03 ];
Tc_error_10  = [ 1.279919e-01 ; 8.863025e-02 ; 8.870291e-01 ];

%-- Image #11:
omc_11 = [ -1.798128e+00 ; -2.081435e+00 ; -4.999265e-01 ];
Tc_11  = [ -1.691940e+02 ; -2.317175e+02 ; 7.034445e+02 ];
omc_error_11 = [ 1.019823e-03 ; 1.091390e-03 ; 2.030933e-03 ];
Tc_error_11  = [ 9.549074e-02 ; 1.969730e-01 ; 8.570220e-01 ];

%-- Image #12:
omc_12 = [ NaN ; NaN ; NaN ];
Tc_12  = [ NaN ; NaN ; NaN ];
omc_error_12 = [ NaN ; NaN ; NaN ];
Tc_error_12  = [ NaN ; NaN ; NaN ];

%-- Image #13:
omc_13 = [ NaN ; NaN ; NaN ];
Tc_13  = [ NaN ; NaN ; NaN ];
omc_error_13 = [ NaN ; NaN ; NaN ];
Tc_error_13  = [ NaN ; NaN ; NaN ];

%-- Image #14:
omc_14 = [ NaN ; NaN ; NaN ];
Tc_14  = [ NaN ; NaN ; NaN ];
omc_error_14 = [ NaN ; NaN ; NaN ];
Tc_error_14  = [ NaN ; NaN ; NaN ];

%-- Image #15:
omc_15 = [ NaN ; NaN ; NaN ];
Tc_15  = [ NaN ; NaN ; NaN ];
omc_error_15 = [ NaN ; NaN ; NaN ];
Tc_error_15  = [ NaN ; NaN ; NaN ];

%-- Image #16:
omc_16 = [ NaN ; NaN ; NaN ];
Tc_16  = [ NaN ; NaN ; NaN ];
omc_error_16 = [ NaN ; NaN ; NaN ];
Tc_error_16  = [ NaN ; NaN ; NaN ];

%-- Image #17:
omc_17 = [ NaN ; NaN ; NaN ];
Tc_17  = [ NaN ; NaN ; NaN ];
omc_error_17 = [ NaN ; NaN ; NaN ];
Tc_error_17  = [ NaN ; NaN ; NaN ];

%-- Image #18:
omc_18 = [ NaN ; NaN ; NaN ];
Tc_18  = [ NaN ; NaN ; NaN ];
omc_error_18 = [ NaN ; NaN ; NaN ];
Tc_error_18  = [ NaN ; NaN ; NaN ];

%-- Image #19:
omc_19 = [ NaN ; NaN ; NaN ];
Tc_19  = [ NaN ; NaN ; NaN ];
omc_error_19 = [ NaN ; NaN ; NaN ];
Tc_error_19  = [ NaN ; NaN ; NaN ];

%-- Image #20:
omc_20 = [ 1.869362e+00 ; 1.584826e+00 ; 1.484195e+00 ];
Tc_20  = [ -1.543202e+02 ; -8.610991e+01 ; 3.950777e+02 ];
omc_error_20 = [ 5.103984e-04 ; 5.501712e-04 ; 6.426442e-04 ];
Tc_error_20  = [ 1.395463e-01 ; 8.552432e-02 ; 6.370338e-01 ];

