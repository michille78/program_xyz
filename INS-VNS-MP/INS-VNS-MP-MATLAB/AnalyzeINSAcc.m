%% xyz 2015.6.24

%% 分析惯性加速度

function AnalyzeINSAcc(  )

clc
clear all
% close all
global dataFolder 
dataFolder = 'E:\data_xyz\Hybrid Motion Capture Data\7.2 dataB\T2';
% dataFolder =  'E:\data_xyz\Hybrid Motion Capture Data\7.6data\1Person3Points_2';
dataName = 'CalData0';
% dataName = 'GlobalAcc';
dataName0 = 'CalData0';
dataName1 = 'CalData1';
% DrawAllINSPosition( dataFolder,dataName0,dataName1,0,15,120,0 );
%% load data
StartK = 1;
[ HeadA,LeftHandA,RightHandA ] = ReadINSAccData( dataFolder,dataName,StartK,2 ) ;

 
% CalStruct = ReadCalData ( dataFolder,dataName ) ;
% 
% HeadA = CalStruct.Head.A ;
% LeftHandA = CalStruct.LeftHand.A ;
% LeftFoot = CalStruct.LeftFoot.A;
% 
% HeadA(3,:) = HeadA(3,:)-1 ;
% LeftHandA(3,:) = LeftHandA(3,:)-1 ;
% LeftFoot(3,:) = LeftFoot(3,:)-1 ;

inertialFre = 120;
inertialN = size(HeadA,2);


%% 搜索头的波峰和波谷
    %% 参数设置
waveThreshold_INSAcc = SetWaveThresholdParameters( 'INSAcc' );
    %% HeadA 输出预设
HeadA_V = NaN(5,inertialN);  
HeadA_WaveFlag = NaN(3,inertialN); 
HeadA_Acc_waveFront = NaN(3,inertialN);
HeadA_Acc_waveBack = NaN(3,inertialN);
HeadA_k_waves_OKLast = zeros(3,1);  % 记录上一时刻判断成功的点（防止判断成功的点被覆盖）

    %% LeftHandA 输出预设
LeftHandA_V = NaN(5,inertialN);  
LeftHandA_WaveFlag = NaN(3,inertialN); 
LeftHandA_Acc_waveFront = NaN(3,inertialN);
LeftHandA_Acc_waveBack = NaN(3,inertialN);
LeftHandA_k_waves_OKLast = zeros(3,1);  % 记录上一时刻判断成功的点（防止判断成功的点被覆盖）

    %% RightHandA 输出预设
RightHandA_V = NaN(5,inertialN);  
RightHandA_WaveFlag = NaN(3,inertialN); 
RightHandA_Acc_waveFront = NaN(3,inertialN);
RightHandA_Acc_waveBack = NaN(3,inertialN);
RightHandA_k_waves_OKLast = zeros(3,1); 

    %% 启动搜索
for j=1:inertialN
    
    [ HeadA_WaveFlag,HeadA_V,HeadA_Acc_waveFront,HeadA_Acc_waveBack,HeadA_k_waves_OKLast ] = AnalyzeWave...
    ( HeadA,j,inertialFre,HeadA_V,HeadA_WaveFlag,HeadA_k_waves_OKLast,HeadA_Acc_waveFront,HeadA_Acc_waveBack,waveThreshold_INSAcc );

    [ LeftHandA_WaveFlag,LeftHandA_V,LeftHandA_Acc_waveFront,LeftHandA_Acc_waveBack,LeftHandA_k_waves_OKLast ] = AnalyzeWave...
    ( LeftHandA,j,inertialFre,LeftHandA_V,LeftHandA_WaveFlag,LeftHandA_k_waves_OKLast,LeftHandA_Acc_waveFront,LeftHandA_Acc_waveBack,waveThreshold_INSAcc );
% 
%     if j>=7757
%         disp('');
%     end
% 
%     [ RightHandA_WaveFlag,RightHandA_V,RightHandA_Acc_waveFront,RightHandA_Acc_waveBack,RightHandA_k_waves_OKLast ] = AnalyzeWave...
%     ( RightHandA,j,inertialFre,RightHandA_V,RightHandA_WaveFlag,RightHandA_k_waves_OKLast,RightHandA_Acc_waveFront,RightHandA_Acc_waveBack,waveThreshold_INSAcc );

    if j==1200
       disp('938'); 
    end
end

if coder.target('MATLAB')
    DrawWaveSearchResult( HeadA,inertialFre,HeadA_V,HeadA_WaveFlag,HeadA_Acc_waveFront,HeadA_Acc_waveBack,'HeadA',inertialN );
     DrawWaveSearchResult( LeftHandA,inertialFre,LeftHandA_V,LeftHandA_WaveFlag,LeftHandA_Acc_waveFront,LeftHandA_Acc_waveBack,'LeftHandA',inertialN );
%     DrawWaveSearchResult( RightHandA,inertialFre,RightHandA_V,RightHandA_WaveFlag,RightHandA_Acc_waveFront,RightHandA_Acc_waveBack,'RightHandA',inertialN );
end



%% 给加速度矢量 增加角度信息

function AccNew = AddAccAngle( Acc )
N = size(Acc,2);
AccNew = NaN(5,N);
AccNew(1:3,:) = Acc;
for k=1:N
    Acc_k = Acc(:,k);
    Acc_xyzNorm_k = normest( Acc_k(1:3) );
    if Acc_xyzNorm_k > 0.1
        Acc_xyNorm_k = normest( Acc_k(1:2) );
        % 水平加速度 跟 东向之间的夹角
        temp1 = atan2( Acc_k(1),Acc_k(2) )*180/pi;
        AccNew(4,k) = temp1;
        % 3D 加速度与地向的夹角
        temp = atan2( Acc_xyNorm_k,Acc_k(3) )*180/pi;
        AccNew(5,k) = temp;
    end
end

function [ HeadA,LeftHandA,RightHandA ] = ReadINSAccData( dataFolder,dataName,startK,zgStatic ) 

CalStruct = ReadCalData ( dataFolder,dataName ) ;

HeadA = CalStruct.Head.A ;
LeftHandA = CalStruct.LeftHand.A ;
RightHandA = CalStruct.RightHand.A;

HeadA(3,:) = HeadA(3,:)-zgStatic ;
LeftHandA(3,:) = LeftHandA(3,:)-zgStatic ;
RightHandA(3,:) = RightHandA(3,:)-zgStatic ;

HeadA = CutData( HeadA,startK );
LeftHandA = CutData( LeftHandA,startK );
RightHandA = CutData( RightHandA,startK );

function data = CutData( data,startK )
[M,N] = size(data);
data = data(:,startK:N);



