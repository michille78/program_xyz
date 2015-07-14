%% xyz  7.2 2015

%% 读取 2个软件分布采集的惯性和视觉数据  识别右手的挥手动作 进行同步


function  ReadAndSyn_INSVNS(  )

clc
clear all
% close all
global dataFolder 
%  dataFolder = 'E:\data_xyz\Hybrid Motion Capture Data\6.25\摔头1';
dataFolder = 'E:\data_xyz\Hybrid Motion Capture Data\7.2 dataB\T2';
dataName = 'CalData1';

%% load data
CalStruct = ReadCalData ( dataFolder,dataName ) ;

HeadA = CalStruct.Head.A ;
LeftHandA = CalStruct.LeftHand.A ;
RightHand = CalStruct.RightHand.A;

HeadA(3,:) = HeadA(3,:)-1 ;
LeftHandA(3,:) = LeftHandA(3,:)-1 ;
RightHand(3,:) = RightHand(3,:)-1 ;

startK = 937;
HeadA = CutData( HeadA,startK );
LeftHandA = CutData( LeftHandA,startK );
RightHand = CutData( RightHand,startK );

inertialFre = 120;
inertialN = size(HeadA,2);



otherMakers = ReadOptitrack( dataFolder,'\Opt.txt' );

startK = 466;
otherMakers = CutData( otherMakers,startK );

otherMakers = FullotherMakersField( otherMakers );

visualN = size(otherMakers,2);
visualFre = otherMakers(1).frequency;
time = (1:inertialN)/inertialFre;

%%

MinMatchDegree = 0.6



function data = CutData( data,startK )
[M,N] = size(data);
data = data(:,startK:N);
