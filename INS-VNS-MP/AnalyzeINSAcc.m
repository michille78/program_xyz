%% xyz 2015.6.24

%% 分析惯性加速度

function AnalyzeINSAcc(  )

clc
clear all
% close all
global dataFolder 
 dataFolder = 'E:\data_xyz\Hybrid Motion Capture Data\6.25\摔头1';
% dataFolder = 'E:\data_xyz\Hybrid Motion Capture Data\6.25\头-综合';
dataName = 'CalculationData';
% dataName = 'GlobalAcc';

%% load data
CalStruct = ReadCalData ( dataFolder,dataName ) ;

HeadA = CalStruct.Head.A ;
LeftHandA = CalStruct.LeftHand.A ;
LeftFoot = CalStruct.LeftFoot.A;

HeadA(3,:) = HeadA(3,:)-1 ;
LeftHandA(3,:) = LeftHandA(3,:)-1 ;
LeftFoot(3,:) = LeftFoot(3,:)-1 ;

inertialFre = 96;
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

    %% 启动搜索
for j=1:inertialN
    [ HeadA_WaveFlag,HeadA_V,HeadA_Acc_waveFront,HeadA_Acc_waveBack,HeadA_k_waves_OKLast ] = AnalyzeWave...
    ( HeadA,j,inertialFre,HeadA_V,HeadA_WaveFlag,HeadA_k_waves_OKLast,HeadA_Acc_waveFront,HeadA_Acc_waveBack,waveThreshold_INSAcc );

    [ LeftHandA_WaveFlag,LeftHandA_V,LeftHandA_Acc_waveFront,LeftHandA_Acc_waveBack,LeftHandA_k_waves_OKLast ] = AnalyzeWave...
    ( LeftHandA,j,inertialFre,LeftHandA_V,LeftHandA_WaveFlag,LeftHandA_k_waves_OKLast,LeftHandA_Acc_waveFront,LeftHandA_Acc_waveBack,waveThreshold_INSAcc );

%     %% 计算速度
%    [ Velocity_k,k_calV ] = CalVelocity( HeadA,j,inertialFre,0.15,0.1,2 ) ;
%    if k_calV>0  && ~isnan(Velocity_k(1))
%         HeadA_V(:,k_calV) = Velocity_k ;
%    end
%    %% 判断 第 k_wave_i = k_calV - adjacentN; 个点的 波峰波谷特征
%    [ WaveFlag_k,k_waves,data_Acc_k_wave ] = FindCrestThrough( HeadA,inertialFre,HeadA_V,k_calV,adjacentN,...
%        waveThreshold_Min_dataA,MinWaveData );
%    % 第 k_waves(i) 个点判断成功后，后续的 adjacentN 个点就不再判断（不然可能会出现误判反而覆盖的现象）
%    for i=1:3       
%        if k_waves(i)>0 && k_waves(i)~=k_waves_OKLast(i)
%            HeadA_WaveFlag(i,k_waves(i)) = WaveFlag_k(i);
%            data_Acc_wave(i,k_waves(i)) = data_Acc_k_wave(i);
%        end
%        % 记录上一时刻 判断成功的位置
%        if ~isnan(WaveFlag_k(i))
%            k_waves_OKLast(i) = k_waves(i);
%        end
%    end
end

if coder.target('MATLAB')
    DrawWaveSearchResult( HeadA,inertialFre,HeadA_V,HeadA_WaveFlag,HeadA_Acc_waveFront,HeadA_Acc_waveBack,'HeadA',inertialN );
    DrawWaveSearchResult( LeftHandA,inertialFre,LeftHandA_V,LeftHandA_WaveFlag,LeftHandA_Acc_waveFront,LeftHandA_Acc_waveBack,'LeftHandA',inertialN );
end


