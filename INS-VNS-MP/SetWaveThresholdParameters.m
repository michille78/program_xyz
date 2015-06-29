%% xyz 2015.6.28

%% 波形分析的参数设置
% dataStyle： 待分析的数据类型

function waveThreshold = SetWaveThresholdParameters( dataStyle )
switch dataStyle
    case 'INSAcc' 
        waveThreshold.adjacentT = 0.09;  %  波峰波谷判断延迟时间
        waveThreshold.waveThreshold_Min_dataA  = 30;  %   波峰/波谷 data_V 的斜率（即data的加速度）最小值
        waveThreshold.MinWaveData = 0.3;   % 波峰波谷处 abs(data) 最小值
        waveThreshold.dT_CalV = 0.15;       % 速度计算的步长时间
        waveThreshold.MinXYVNorm_CalAngle = 2;  % 计算xy速度方向要求的最小xy速度模值  
        
    case 'VNSAcc'
        waveThreshold.adjacentT = 0.07;  %  波峰波谷判断延迟时间
        waveThreshold.waveThreshold_Min_dataA  = 20;  %   波峰/波谷 data_V 的斜率（即data的加速度）最小值
        waveThreshold.MinWaveData = 0.3;   % 波峰波谷处 abs(data) 最小值
        waveThreshold.dT_CalV = 0.15;       % 速度计算的步长时间
        waveThreshold.MinXYVNorm_CalAngle = 2;  % 计算xy速度方向要求的最小xy速度模值  
end

