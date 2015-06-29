%% xyz  2015.6.28

%% 波峰波谷搜索 （实时）
%% 更新了data的第 k_data 数时，搜索一步波峰波谷
% data [3*N]  待搜索的曲线
% k_data   % data 最新更新数据的序号
% inertialFre   % data 的频率
% dataV  data  % 的微分结果
% data_WaveFlag  %  最后的波形判断结果。1>NaN 无波形特征  2> 该点的data值，为正则为波峰，为负则为波谷。
% k_waves_OKLast  % 记录上一时刻判断成功的点（防止判断成功的点被覆盖）
% dataA_waveFront   % 记录dataV越0线的点的 左 平均dataV斜率（为waveThreshold_Min_dataA的设置提高参考）
% dataA_waveBack   % 记录dataV越0线的点的 右 平均dataV斜率（为waveThreshold_Min_dataA的设置提高参考）
% waveThreshold 判定参数设置
        % .adjacentN  从最新的速度 k_calV 基础上，波峰波谷判断延迟个数。 [k_calV-adjacentN:k_calV+adjacentN]用于判断 dataV 的斜率是否够。
        % .waveThreshold_Min_dataA  波峰/波谷 data_V 的斜率（即data的加速度）最小值
        % .MinWaveData  波峰波谷处 abs(data) 最小值


function [ data_WaveFlag,dataV,dataA_waveFront,dataA_waveBack,k_waves_OKLast ] = AnalyzeWave...
    ( data,k_data,fre,dataV,data_WaveFlag,k_waves_OKLast,dataA_waveFront,dataA_waveBack,waveThreshold )

%% 预设输出

%% 读参数设置
adjacentT = waveThreshold.adjacentT;
adjacentN = fix(adjacentT*fre);
waveThreshold_Min_dataA = waveThreshold.waveThreshold_Min_dataA;
MinWaveData = waveThreshold.MinWaveData;
dT_CalV = waveThreshold.dT_CalV;
MinXYVNorm_CalAngle = waveThreshold.MinXYVNorm_CalAngle;
 %% 计算速度
   [ Velocity_k,k_calV ] = CalVelocity( data,k_data,fre,dT_CalV,MinXYVNorm_CalAngle,2 ) ; % 采用法2求速度
   if k_calV>0  && ~isnan(Velocity_k(1))
        dataV(:,k_calV) = Velocity_k ;
   end
   %% 判断 第 k_wave_i = k_calV - adjacentN; 个点的 波峰波谷特征
   [ WaveFlag_k,k_waves,data_Acc_k_wave ] = FindCrestThrough( data,fre,dataV,k_calV,adjacentN,...
       waveThreshold_Min_dataA,MinWaveData );
   % 第 k_waves(i) 个点判断成功后，后续的 adjacentN 个点就不再判断（不然可能会出现误判反而覆盖的现象）
   for i=1:3       
       if k_waves(i)>0 && k_waves(i)~=k_waves_OKLast(i)   % 不是之前判断OK的点
           data_WaveFlag(i,k_waves(i)) = WaveFlag_k(i);
           dataA_waveFront(i,k_waves(i)) = data_Acc_k_wave(i,1);
           dataA_waveBack(i,k_waves(i)) = data_Acc_k_wave(i,2);
           %% 寻找波峰波谷中间点
           if ~isnan(WaveFlag_k(i)) && k_waves_OKLast(i)>0 
               % 波峰波谷相邻
               k1 = k_waves_OKLast(i);
               k2 = k_waves(i);
              if  size(data,2)>=k1 && ~isnan(data(i,k1)) && sign(data(i,k1)) * sign( data(i,k2) ) == -1
                    % 在 k_waves_OKLast(i) 到 k_waves(i) 搜索中间点                                   
                    dataV_Search = dataV(i,k1+1:k2-1);
                    % 要求 dataV_Search 全大于0  或全小于0
                    if abs(sum(sign(dataV_Search))) >= k2-k1-2+1-2
                        dataSearch = abs(data(i,k1:k2));
                        [min_dataAbs,min_k] = min(dataSearch,[],2);
                        if i==3
                           disp('test') 
                        end
                        data_WaveFlag(i,min_k+k1-1) = 0;   %  中间点 搜索OK
                    end
              end
           end
       end
       
       
       % 更新上一时刻 判断成功的位置
       if ~isnan(WaveFlag_k(i))
           k_waves_OKLast(i) = k_waves(i);
       end
   end