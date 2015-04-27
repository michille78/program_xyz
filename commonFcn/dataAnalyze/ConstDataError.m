%% xyz 2015.3.16
% analyze the error of single const data
% output: mean, std
function [ meanD,stdD,maxErr1,maxErr2 ] = ConstDataError( constD )

meanD = mean(constD);
stdD = std(constD);
maxErr1 = max(constD)-meanD ;
maxErr2 = meanD-min(constD) ;
