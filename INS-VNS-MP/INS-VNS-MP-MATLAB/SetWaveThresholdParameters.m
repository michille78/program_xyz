%% xyz 2015.6.28

%% 波形分析的参数设置
% dataStyle： 待分析的数据类型

function waveThreshold = SetWaveThresholdParameters( dataStyle )
switch dataStyle
    case 'INSAcc' 
        waveThreshold.adjacentT = 0.15;  %  波峰波谷判断延迟时间（决定速度斜率计算区间，取越大，速度斜率的额平滑程度越大，波出现的阈值也越小）
        waveThreshold.waveThreshold_Min_dataA  = 6;  %   波峰/波谷 data_V 的斜率（即data的加速度）最小值
        waveThreshold.MinWaveData = 0.1;   % 波峰波谷处 abs(data) 最小值
        waveThreshold.dT_CalV = 0.1;       % 速度计算的步长时间
        waveThreshold.MinXYVNorm_CalAngle = 0.5;  % 计算xy速度方向要求的最小xy速度模值  
        waveThreshold.FullWaveDataScope = 1 ;     % 100% 波强度时的 data 范围
    case 'VNSAcc'
        waveThreshold.adjacentT = 0.15;  %  波峰波谷判断延迟时间（决定速度斜率计算区间，取越大，速度斜率的额平滑程度越大）
        waveThreshold.waveThreshold_Min_dataA  = 15;  %   波峰/波谷 data_V 的斜率（即data的加速度）最小值
        waveThreshold.MinWaveData = 0.4;   % 波峰波谷处 abs(data) 最小值
        waveThreshold.dT_CalV = 0.1;       % 速度计算的步长时间
        waveThreshold.MinXYVNorm_CalAngle = 2;  % 计算xy速度方向要求的最小xy速度模值  
        waveThreshold.FullWaveDataScope = 5 ;     % 100% 波强度时的 data 上下差 范围
end

