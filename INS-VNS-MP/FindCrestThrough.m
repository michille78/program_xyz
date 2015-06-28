%% xyz 2015.6.26

%% 寻找波峰波谷 和 峰谷的中间点 (实时)
% 刚更新了 第 k_calV 个点的速度
% 判断 第 k_wave_i = k_calV - adjacentN; 个点的 波峰波谷特征
        % 1) 判断 data_V(:,k_calV) 是不是刚刚越过 0 线，是则下一步
        % 2）判断这个点左边5个点的 data_V 斜率 和右边5个点的 data_V 斜率 是否同号且足够大，够大则认为对应一个波峰/谷。
        % 3）在这个点附近 5 个点 找到最大或最小的data，则为波峰/谷。（因为速度的计算会有延迟或滞后）
        % 4) 要求波峰或波谷处的 data 绝对值大于 MinWaveData

% data [3*N]  原始数据
% data_V [5*N]  速度信息
% fre  频率 HZ
% waveThreshold_Min_dataA  波峰/波谷 data_V 的斜率（即data的加速度）最小值
%  adjacentN = 4;  %  波峰波谷判断延迟个数
% MinWaveData % 波峰波谷处 abs(data) 最小值

% k_waves  寻找的波峰/波谷 位置 
% WaveFlag_k 寻找失败为 NaN,成功则返回该点的 data（WaveFlag_k>0是波峰，<0是波谷）

%% 通过 WaveFlag_k 是否为 NaN 判断是否波峰/波谷


function [ WaveFlag_k,k_waves,data_Acc_k_wave ] = FindCrestThrough( data,fre,data_V,k_calV,adjacentN,waveThreshold_Min_dataA,MinWaveData )

% data_V = NaN(5,N);  
% 
% for j=1:N
%     %% 计算速度
%    [ Velocity_k,k_calV ] = CalVelocity( data,j,fre,dT_CalV,MinXYVNorm_CalAngle,2 ) ;
%    if k_calV>0  && ~isnan(Velocity_k(1))
%         data_V(:,k_calV) = Velocity_k ;
%    end
% end

WaveFlag_k = NaN(3,1);    % 所以波峰和波谷的加速度值

k_wave = k_calV - adjacentN;  % 待判断的点
k_waves = ones(3,1)*k_wave;  % 每一维度进过细调后波的位置可能不一样
data_Acc_k_wave = NaN(3,1);

if k_wave<2 || isnan(data(1,k_wave)) || isnan(data_V(1,k_wave)) || isnan(data_V(1,k_wave-1))
   return; 
end

if k_wave==403
   disp(''); 
end

for i=1:3
    k_wave_i = k_wave;
    % 1) 判断 data_V(:,k_calV) 是不是刚刚越过 0 线，是则下一步
    if sign( data_V(i,k_wave_i) )*sign( data_V(i,k_wave_i-1) ) < 1  
        if( abs(data_V(i,k_wave_i)) > abs(data_V(i,k_wave_i-1)) )  % 找2个值中的小者
            k_wave_i = k_wave_i-1;
        end
        % 计算 k_wave_i 点 前后 adjacentN 个点区间 的平均斜率
        data_V_k_wave = data_V(i,k_wave_i);
        data_V_Back = data_V(i,k_calV);     % 斜率计算区间 最后一个点
        data_V_Front = data_V(i,k_calV-adjacentN*2);  % 斜率计算区间 最前一个点
        if isnan(data_V_Back) || isnan(data_V_Front) || isnan(data_V_k_wave)
            return;
        end
        data_Acc1 = (data_V_Back-data_V_k_wave)/(k_calV-k_wave_i)*fre ;  % 后面一段的斜率
        data_Acc2 = (data_V_k_wave-data_V_Front)/(k_wave_i-k_calV+adjacentN*2)*fre ;  % 前面一段的斜率
        % 要求 前面和后面一段data_V的斜率 同号 且都比较大
        if sign(data_Acc1)*sign(data_Acc2) > 0
            % 2) 波峰/谷 存在 判断 OK， 进行 3) 在 k_wave_i 附近3个点内 细调位置
            data_Acc_k_wave(i) = (data_Acc1+data_Acc2)/2;  % 用于输出查看
           if abs(data_Acc1) > waveThreshold_Min_dataA && abs(data_Acc2) > waveThreshold_Min_dataA              
              % 3）在这个点附近 5 个点 找到最大或最小的data，则为波峰/谷。（因为速度的计算会有延迟或滞后）
              data_temp = data(i,k_wave_i-2:k_wave_i+2) ;   
              if isnan( sum(data_temp) )
                  return;
              end
              if data(i,k_wave_i)>0
                  % 波峰 细调
                  [ data_max,k_max ] = max(data_temp,[],2);
                  data_m = data_max;
                  k_wave_i = k_wave_i+(k_max-3);                  
              else
                  % 波谷 细调
                  [ data_min,k_min ] = min(data_temp,[],2);
                  data_m = data_min;
                  k_wave_i = k_wave_i+(k_min-3);
              end
              %% 1) 2) 3) 判断OK
              %% 4) 要求波峰或波谷处的 data 绝对值大于 MinWaveData
              if abs(data_m)>MinWaveData
                  %% 搜索OK 记录波峰/波谷的 data
                    k_waves(i) = k_wave_i;
                    WaveFlag_k(i,1) = data(i,k_wave_i); 
              end              
           end
        end
        
    end    
end
        



