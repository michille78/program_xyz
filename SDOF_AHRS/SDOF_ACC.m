%% analysis the magnitude of acceleration in SDOF motion

function SDOF_ACC()

clc
clear all
close all
%% ’˝œ“‘À∂Ø

L = 1 ;
afa = 60*pi/180;
f = 0.05 ;

w = 2*pi*f*afa  ;
w_d = w*180/pi
dw = (2*pi*f)^2*afa  ;
dw_d = dw*180/pi

 w = 5*pi/180;
 dw = 0*pi/180;

a_n = ( w )^2*L  /9.8*1000
a_t = dw*L   /9.8*1000



