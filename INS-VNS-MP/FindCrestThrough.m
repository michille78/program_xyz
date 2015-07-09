%% xyz 2015.6.26

%% 寻找波峰波谷 和 峰谷的中间点 (实时)
% 刚更新了 第 k_calV 个点的速度
% 判断 第 k_wave_i = k_calV - adjacentN; 个点的 波峰波谷特征
        % 1.1)  data 绝对值大于 MinWaveData  
        % 1.2) 判断 data_V(:,k_calV) 是不是刚刚越过 0 线，是则下一步
        % 2）判断这个点左边5个点的 data_V 斜率 和右边5个点的 data_V 斜率 是否同号且足够大，够大则认为对应一个波峰/谷。
            %    斜率足够大的思路有好几种，这里选择只要左或右有一个足够大就OK。
        % 3）在这个点附近 5 个点 找到最大或最小的data，则为波峰/谷。（因为速度的计算会有延迟或滞后）

% data [3*N]  原始数据
% data_V [5*N]  速度信息
% fre  频率 HZ
% waveThreshold_Min_dataA  波峰/波谷 data_V 的斜率（即data的加速度）最小值
%  adjacentN = 4;  % 从最新的速度 k_calV 基础上，波峰波谷判断延迟个数。 [k_calV-adjacentN:k_calV+adjacentN]用于判断 dataV 的斜率是否够。
% MinWaveData % 波峰波谷处 abs(data) 最小值

% k_waves  寻找的波峰/波谷 位置 
% WaveFlag_k  存储 波特征强度指标 1）<0为波峰，>0为波谷 2) =0 为波腰  3）  NaN 为非波特征; 
% data_Acc_k_wave [3*2] 第一类记录 k_waves 前面的 dataV 平均斜率， 第二列记录后面的
% data 的有效长度
% FullWaveDataScope ：  % 100% 波强度时的 data 范围

%% 通过 WaveFlag_k 是否为 NaN 判断是否波峰/波谷
%% 波特征强度指标
%   1） data 加速度大小
%   2） 波特征前后持续的时间长度
%   3） 波特征 data 范围


function [ WaveFlag_k,k_waves,data_Acc_k_wave ] = FindCrestThrough...
    ( data,fre,data_V,k_calV,waveThreshold,dataValidN )
%% Input
adjacentT = waveThreshold.adjacentT; % 波范围检测的最大值（同时限定 dataV 斜率计算区间）
adjacentN = fix(adjacentT*fre);
waveThreshold_Min_dataA = waveThreshold.waveThreshold_Min_dataA;
MinWaveData = waveThreshold.MinWaveData;

step_calT = min(adjacentT,0.15);  % dataV 斜率计算区间 
step_calN = fix(step_calT*fre);
% dT_CalV = waveThreshold.dT_CalV;
% MinXYVNorm_CalAngle = waveThreshold.MinXYVNorm_CalAngle;
FullWaveDataScope = waveThreshold.FullWaveDataScope;

%% 
WaveFlag_k = NaN(3,1);    % 所以波峰和波谷的加速度值

k_wave = k_calV - adjacentN;  % 待判断的点
k_waves = ones(3,1)*k_wave;  % 每一维度进过细调后波的位置可能不一样
data_Acc_k_wave = NaN(3,2);


if k_wave < adjacentN
    return;
end

for i=1:3
    k_wave_i = k_wave;
        
    % 1.1)  data 与附近10个的平均值 的差 绝对值大于 MinWaveData  
    if abs( data(i,k_wave_i) )< MinWaveData
        continue;
    end
    % 1.2) 判断 data_V(:,k_calV) 是不是刚刚越过 0 线，是则下一步
    if sign( data_V(i,k_wave_i) )*sign( data_V(i,k_wave_i-1) ) < 1  
        

        
        if( abs(data_V(i,k_wave_i)) > abs(data_V(i,k_wave_i-1)) )  % 找2个值中的小者
            k_wave_i = k_wave_i-1;
        end
        % 计算 k_wave_i 点 前后 adjacentN 个点区间 的平均斜率
        data_V_k_wave = data_V(i,k_wave_i);
        
        % 搜索 data_V 斜率计算区间  [ front_k_calA  k_wave_i  back_k_calA   ]
        step_calA = min( k_calV-k_wave_i,step_calN );
        % 不允许在斜率计算区间  [ front_k_calA  k_wave_i  back_k_calA   ]内出现速度拐点
        [ front_k_calA,back_k_calA ] = CheckCalAStep( data_V,k_wave_i,step_calA,i );         
        if isempty(front_k_calA) || isempty(back_k_calA)
            return;
        end
        
        data_V_Back = data_V(i,back_k_calA);     % 斜率计算区间 最后一个点
        data_V_Front = data_V(i,front_k_calA);  % 斜率计算区间 最前一个点
        if isnan(data_V_Back) || isnan(data_V_Front) || isnan(data_V_k_wave)
            return;
        end
        data_Acc_Back = (data_V_Back-data_V_k_wave)/(k_calV-k_wave_i)*fre ;  % 后面一段的斜率
        data_Acc_Front = (data_V_k_wave-data_V_Front)/(k_wave_i-k_calV+adjacentN*2)*fre ;  % 前面一段的斜率
        
        data_Acc_k_wave(i,1) = data_Acc_Front;   % 记录下前后的 data_V 斜率
        data_Acc_k_wave(i,2) = data_Acc_Back;
        k_waves(i) = k_wave_i;
        
        %% 通过波特征指标判断 是否 为有效的波特征
        % 1） data 左右 加速度 data_Acc 均够大
        % 2） 波覆盖的时间范围 timeScope 够大
        % 3） 波覆盖的data值范围 dataScope 够大
        if sign(data_Acc_Front)*sign(data_Acc_Back) > 0    % 首先要求同号，<0为波峰，>0为波谷
            waveSign = sign(data_Acc_Front);  % -1 为波峰，1为波谷
                       
            data_Acc_k_absMax = max( abs(data_Acc_Back),abs(data_Acc_Front) );
            data_Acc_k_absMin = min( abs(data_Acc_Back),abs(data_Acc_Front) );
            data_Acc_k_absMean = ( abs(data_Acc_Back)+abs(data_Acc_Front) )/2;
           if data_Acc_k_absMax > waveThreshold_Min_dataA &&  data_Acc_k_absMin > waveThreshold_Min_dataA/3
               
                % 搜索附近的最大值
                MaxSearchN = 5;
                dataAbsMax = abs(data(i,k_wave_i));
                k_wave_i_Final =  k_wave_i;
                for k = k_wave_i-MaxSearchN:min(k_wave_i+MaxSearchN,dataValidN)
                    dataCur = data(i,k);
                    if abs(dataCur) > dataAbsMax
                        dataAbsMax = abs(dataCur);
                        k_wave_i_Final = k;
                    end
                end
                k_wave_i =  k_wave_i_Final;
                
            %% 分析波特征强度
            waveDegree = CalWaveFeatureDegree...
                ( data,data_V,k_wave_i,i,fre,k_calV,dataValidN,adjacentT,FullWaveDataScope,waveThreshold_Min_dataA,data_Acc_k_absMean,waveSign );
            
%             % 搜索 前面速度=0的点（波的开始），后面速度为0的点（波的结束）
%                FullSearchT = 0.6;   % 100% 波强度时的 time 范围
%                MaxSearchN = fix(FullSearchT/2*fre);  % 最大搜索范围
%                k_wave_Start = k_wave_i-3;
%                step = 1;
%                for s = 3:MaxSearchN
%                    k = k_wave_i-s;
%                    
%                    if ~isnan(data_V(i,k-1))  % 优先用 dataV 搜索法，无速度数据则用 data 搜索法
%                        %% dataV搜索法（优点：可不受INS噪声影响，因为速度是被平滑的。缺点：视觉的连续曲线比较短，可能速度没有数值）
%                        if k-1 < 1  % 没有数据，结束搜索
%                            break;
%                        end
%                        dataVk1 = data_V(i,k);
%                        dataVk2 = data_V(i,k-1);
%                        if sign( dataVk1 ) * sign( dataVk2 ) < 1 % 速度变号                                             
%                             break;% 即最后的波特征 起点   
%                        end
%                        k_wave_Start = k; 
%                    else
%                        %% data搜索法（优点：受data噪声影响，但对视觉数据问题不大） 
%                        if k-2*step < 1  % 没有数据，结束搜索
%                            break;
%                        end
%                        datak1 = data(i,k);
%                        datak2 = data(i,k-1*step);
%                        datak3 = data(i,k-2*step);
% 
%                        WaveStartFlag = sign( datak1-datak2 ) * sign( datak2-datak3 );
%                        if WaveStartFlag ~= 1                         
%                           break; % 搜索到 data 拐点
%                        end
%                        k_wave_Start = k;
%                    end
%                end
%                % 搜索波的结束
%                k_wave_Stop = k_wave_i+3;
%                step = 1;
%                for s = 6:MaxSearchN
%                    k = k_wave_i+s;
%                    if ~isnan(data_V(i,k-1))   % 优先用 dataV 搜索法，无速度数据则用 data 搜索法
%                        %% dataV搜索法
%                        if k>k_calV
%                           continue; 
%                        end
%                        dataVk1 = data_V(i,k);
%                        dataVk2 = data_V(i,k-1);
%                        if sign( dataVk1 ) * sign( dataVk2 ) < 1 % 速度遇到拐点                                       
%                             break;% 即最后的波特征 结束点        
%                        end
%                        k_wave_Stop = k; 
%                    else
%                        %% data 搜索法
%                        if k+2*step > dataValidN  % 没有数据，结束搜索
%                            break;
%                        end                   
%                        datak1 = data(i,k);
%                        datak2 = data(i,k+1*step);
%                        datak3 = data(i,k+2*step);                   
%                        WaveStartFlag = sign( datak1-datak2 ) * sign( datak2-datak3 );
%                        if WaveStartFlag ~= 1                         
%                           break; % 搜索到 data 拐点
%                        end
%                        k_wave_Stop = k;
%                    end
%                end
%                %% 波特征范围
%                waveTimeScope = (k_wave_Stop-k_wave_Start)/fre ;
%                waveDataScopeFront = data(i,k_wave_i)-data(i,k_wave_Start) ;
%                waveDataScopeBack = data(i,k_wave_i)-data(i,k_wave_Stop) ;
%                waveDataScopeMean = (abs(waveDataScopeFront)+abs(waveDataScopeBack))/2;
%                % 综合3个指标给出波特征的强度
%                waveDegree1 = waveTimeScope/(adjacentT*2) ;
%                waveDegree2 = waveDataScopeMean/FullWaveDataScope ;
%                waveDegree3 = data_Acc_k_absMean/(waveThreshold_Min_dataA*4); % 认为最小值的 3 倍即为 100% 水平
%                waveDegree = waveSign * ( waveDegree1*0.2 + waveDegree2*0.6 + waveDegree3*0.2 )  ;
               %% 波的强度大于阈值则判断有效
               if abs(waveDegree) > 0.1
                    k_waves(i) = k_wave_i;
                    WaveFlag_k(i) = waveDegree; 
               end
           end
        end
        
%         % 要求 前面和后面一段data_V的斜率 同号 且都比较大
%         if sign(data_Acc_Front)*sign(data_Acc_Back) > 0  % 首先要求同号，<0为波峰，>0为波谷
%             % 1） data 左右 加速度 data_Acc 均够大
%             % 2） 波覆盖的时间范围 timeScope 够大
%             % 3） 波覆盖的data值范围 dataScope 够大
%             WaveSign = sign(data_Acc_Front);  % <0为波峰，>0为波谷
%             
%             data_Acc_k_absMax = max( abs(data_Acc_Back),abs(data_Acc_Front) );
%             data_Acc_k_absMin = min( abs(data_Acc_Back),abs(data_Acc_Front) );
%            if data_Acc_k_absMax > waveThreshold_Min_dataA &&  data_Acc_k_absMin > waveThreshold_Min_dataA/3
%                               
%                % 2) 波峰/谷 存在 初步判断 OK， 进行 3) 在 k_wave_i 附近3个点内 细调位置
%                % 3）在这个点附近 5 个点 找到最大或最小的data，则为波峰/谷。（因为速度的计算会有延迟或滞后）
%               data_temp = data(i,k_wave_i-2:k_wave_i+2) ;   
%               if isnan( sum(data_temp) )
%                   return;
%               end
%               if data(i,k_wave_i)>0
%                   % 波峰 细调
%                   [ data_max,k_max ] = max(data_temp,[],2);
%                   data_m = data_max;
%                   k_wave_i = k_wave_i+(k_max-3);                  
%               else
%                   % 波谷 细调
%                   [ data_min,k_min ] = min(data_temp,[],2);
%                   data_m = data_min;
%                   k_wave_i = k_wave_i+(k_min-3);
%               end
%               
%               % 搜索 前面速度=0的点（波的开始），后面速度为0的点（波的结束）
%                MaxSearchN = fix(0.2*fre);  % 最大搜索范围
%                for k=3:MaxSearchN
%                    k_waveStart =  k_wave_i-k;
%                    dataV_Temp = data_V(i,k_waveStart) ;
%                    
%                end
%                
%               %% 1) 2) 3) 判断OK
%                   %% 搜索OK 记录波峰/波谷的 data
%                     k_waves(i) = k_wave_i;
%                     WaveFlag_k(i,1) = data(i,k_wave_i); 
%               
%            end
%         end
        
    end    
end


function  waveDegree = CalWaveFeatureDegree...
    ( data,data_V,k_wave_i,i,fre,k_calV,dataValidN,adjacentT,FullWaveDataScope,waveThreshold_Min_dataA,data_Acc_k_absMean,waveSign )

%% 分析波特征强度
% 搜索 前面速度=0的点（波的开始），后面速度为0的点（波的结束）
FullSearchT = 0.6;   % 100% 波强度时的 time 范围
MaxSearchN = fix(FullSearchT/2*fre);  % 最大搜索范围
k_wave_Start = k_wave_i-3;
step = 1;
for s = 3:MaxSearchN
   k = k_wave_i-s;

   if ~isnan(data_V(i,k-1))  % 优先用 dataV 搜索法，无速度数据则用 data 搜索法
       %% dataV搜索法（优点：可不受INS噪声影响，因为速度是被平滑的。缺点：视觉的连续曲线比较短，可能速度没有数值）
       if k-1 < 1  % 没有数据，结束搜索
           break;
       end
       dataVk1 = data_V(i,k);
       dataVk2 = data_V(i,k-1);
       if sign( dataVk1 ) * sign( dataVk2 ) < 1 % 速度变号                                             
            break;% 即最后的波特征 起点   
       end
       k_wave_Start = k; 
   else
       %% data搜索法（优点：受data噪声影响，但对视觉数据问题不大） 
       if k-2*step < 1  % 没有数据，结束搜索
           break;
       end
       datak1 = data(i,k);
       datak2 = data(i,k-1*step);
       datak3 = data(i,k-2*step);

       WaveStartFlag = sign( datak1-datak2 ) * sign( datak2-datak3 );
       if WaveStartFlag ~= 1                         
          break; % 搜索到 data 拐点
       end
       k_wave_Start = k;
   end
end
% 搜索波的结束
k_wave_Stop = k_wave_i+3;
step = 1;
for s = 6:MaxSearchN
   k = k_wave_i+s;
   if ~isnan(data_V(i,k-1))   % 优先用 dataV 搜索法，无速度数据则用 data 搜索法
       %% dataV搜索法
       if k>k_calV
          continue; 
       end
       dataVk1 = data_V(i,k);
       dataVk2 = data_V(i,k-1);
       if sign( dataVk1 ) * sign( dataVk2 ) < 1 % 速度遇到拐点                                       
            break;% 即最后的波特征 结束点        
       end
       k_wave_Stop = k; 
   else
       %% data 搜索法
       if k+2*step > dataValidN  % 没有数据，结束搜索
           break;
       end                   
       datak1 = data(i,k);
       datak2 = data(i,k+1*step);
       datak3 = data(i,k+2*step);                   
       WaveStartFlag = sign( datak1-datak2 ) * sign( datak2-datak3 );
       if WaveStartFlag ~= 1                         
          break; % 搜索到 data 拐点
       end
       k_wave_Stop = k;
   end
end
%% 波特征范围
waveTimeScope = (k_wave_Stop-k_wave_Start)/fre ;
waveDataScopeFront = data(i,k_wave_i)-data(i,k_wave_Start) ;
waveDataScopeBack = data(i,k_wave_i)-data(i,k_wave_Stop) ;
waveDataScopeMean = (abs(waveDataScopeFront)+abs(waveDataScopeBack))/2;
% 综合3个指标给出波特征的强度
waveDegree1 = waveTimeScope/(adjacentT*2) ;
waveDegree2 = waveDataScopeMean/FullWaveDataScope ;
waveDegree3 = data_Acc_k_absMean/(waveThreshold_Min_dataA*4); % 认为最小值的 3 倍即为 100% 水平
waveDegree = waveSign * ( waveDegree1*0.2 + waveDegree2*0.6 + waveDegree3*0.2 )  ;               

%% 不允许在斜率计算区间  [ front_k_calA  k_wave_i  back_k_calA   ]内出现速度拐点
function [ front_k_calA,back_k_calA ] = CheckCalAStep( data_V,k_wave_i,step_calA,i )

front_k_calA=[];
back_k_calA=[];

for s = 3:step_calA
    fk = k_wave_i-s;    
    dataCur = data_V(i,fk);      
    dataFront = data_V(i,fk-1);
    dataBack = data_V(i,fk+1);
    if isnan( dataCur ) || isnan( dataFront )  || isnan( dataBack ) 
       break ;  % 没有足够数据
    end
    IsExtremum = sign( dataCur-dataFront ) * sign(dataBack-dataCur);  % 符号相反则为极值点                
    if IsExtremum == -1 || IsExtremum==0 % 极值点                     
        break;
    end
    
    front_k_calA = fk;
end

for s = 3:step_calA
    bk = k_wave_i+s;    
    dataCur = data_V(i,bk);    
    dataFront = data_V(i,bk-1);
    dataBack = data_V(i,bk+1);
    if isnan( dataCur ) || isnan( dataFront )  || isnan( dataBack ) 
       break ;  % 没有足够数据
    end
    IsExtremum = sign( dataCur-dataFront ) * sign(dataBack-dataCur);  % 符号相反则为极值点                
    if IsExtremum == -1 || IsExtremum==0 % 极值点                     
        break;
    end
    back_k_calA = bk;
end


               

function DrawCurWave( data,fre,data_V,dataValidN,WaveFlag_k,k_waves,k_wave_Start,k_wave_Stop )
ftime = 0.05;
figure
time = (1:dataValidN);
dataValid = data(:,1:dataValidN);
data_V_Valid = data_V(:,1:dataValidN);

for i=1:3
    subplot(3,1,i)
    hold on
    plot( time,dataValid(i,:) )
    plot( time,data_V_Valid(i,:),'g' )
    if ~isnan(WaveFlag_k(i))
       plot(time(k_waves(i)),dataValid(i,k_waves(i)),'*r') 
       plot( time(k_wave_Start),dataValid(i,k_wave_Start),'or' )
       plot( time(k_wave_Stop),dataValid(i,k_wave_Stop),'or' )
       text( time(k_waves(i))-ftime,dataValid(i,k_waves(i)),sprintf('%0.2f',WaveFlag_k(i)) )
    end
end
