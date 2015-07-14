%% 读取 惯性 和 视觉 数据， 调用 Cpp 的 GetINSCompensateFromVNS

function Cpp_OptitrackIneritalFusion()

dataFolder = 'E:\data_xyz\Hybrid Motion Capture Data\5.28\5.28-head1';

%% load data

S = coder.load( 'InertialData.mat','InertialData' );


% S = coder.load( 'otherMakers.mat','otherMakers' );
% 
% S = coder.load('E:\data_xyz\Hybrid Motion Capture Data\5.28\5.28-head1\InertialData.mat','InertialData' );
% otherMakers = coder.load([dataFolder,'\otherMakers.mat']);
% 
% CalStruct = coder.load( [dataFolder,'\CalData_Avatar00.mat'] );
% 
% BVHStruct = coder.load( [dataFolder,'\BVHData.mat'] );


