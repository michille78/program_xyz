%% xyz 2015.4.13

% magnetic calibration parameters of n frame to b frame


function [ G_b,b_b ] =  MagneticCalibration_n_To_b( G_n,b_n,Cnb )
Cbn = Cnb' ;
G_b = Cnb * G_n * Cbn ;
b_b = Cnb * b_n ;

