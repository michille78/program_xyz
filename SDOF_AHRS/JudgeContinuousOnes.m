%% xyz 2015.4.23   2015.5.12

%% 提取IsOnes中连续为1的部分（可改进为实时）
% 目的：提取陀螺仪测得的角速度=0 且 变化率=0 的时间
% 方法： IsGyroNormZero 长时间=0 的时间段
%   1）找到时间长度>minContinuesN 的 IsGyroNormZero=1 数据区段：可能为角加速度为0
%   2）找到该区段附近，陀螺数据变化最剧烈的分量，对该分量，利用鲁棒方法拟合该区段数据为一条直线（RANSA、LMedS）
%   3）根据拟合后得到直线的斜率判断是否为 长时间IsGyroNormZero=1
%   4）对于满足以上条件的区段，剔除离群点后再次平滑，得到最终 长时间IsGyroNormZero=1 的结果
%%% INput
% IsOnes: [1*N]或[N*1] 已经判断好的 且平滑后的（不平滑会判断出错！） 是否为1的结果
function IsContinuousOnes = JudgeContinuousOnes( IsOnes,GyroData,frequency,minContinuesN,maxAngularAcc )

AbandomN = fix(0.02*frequency) ;     % 判断角加速度=0 后，区间前后剔除的个数
AbandomN = max(AbandomN,2);

Nframes = length( IsOnes );
IsContinuousOnes = zeros( size(IsOnes) );

% 陀螺数据中变化最剧烈的
MeanGyroData = mean( abs(GyroData),2 );
[ ~,I ] = max(MeanGyroData);
GyroData_1Dim = GyroData(I,:);
winStart = 0;
for k=2:Nframes
    if IsOnes(k)==1
        if IsOnes(k-1)==0 || k==2  
            %　0->1
            winStart = k;            
        end
        
    else
        if IsOnes(k-1)==1
           %  1->0
            if winStart==0
                continue;
            end
            winEnd = k-1;
            M = winEnd-winStart+1 ;
            if M>minContinuesN
                %% 1）找到时间长度>minContinuesN 的 IsGyroNormZero=1 数据区段：可能为角加速度为0
                % 2) 对这段区间的陀螺数据拟合
                gyro = GyroData_1Dim(winStart:winEnd);
                time = (winStart:winEnd)/frequency;
                P = polyfit(time,gyro,1);
                GyroConstFitting = ( P(2)+P(1)* mean(time) ) ;  % 拟合直线在该区段中点的值，理论值为0
                GyroConstFitting_degree = GyroConstFitting*180/pi;
                AngularAcc = P(1)*180/pi;
                % P(1)为角加速度
                if abs(P(1))<maxAngularAcc && GyroConstFitting<1*pi/180  %　maxAngularAcc：角加速度阈值
                   %% 3）判断满足 角速度=0 + 角加速度=0
                   % 4) 剔除拟合过程中残差较大的数据                   
                   IsContinuousOnes(winStart+AbandomN:winEnd-AbandomN) = 1 ;        
                end
            end
        end        
    end
end





% xyz 2015.4.23 

%     %% 判断某个点k领域内连续为1
%     % 1）第k-1点为1,第k点为1,第k+1点为1
%     % 2）第k点前的 frontN 个点有 70% 以上为1
%     % 3）第k点后的 laterN 个点有 70% 以上为1
%     %%% INput
%     % IsOnes: [1*N]或[N*1] 已经判断好的 是否为1的结果
%     function IsContinuousOnes = JudgeContinuousOnes( IsOnes,frontT,laterT,frequency,minRateFront,minRateLater )
%     Nframes = length( IsOnes );
%     IsContinuousOnes = zeros( size(IsOnes) );
%     frontN = max( fix(frontT * frequency),3) ;
%     laterN = max( fix(laterT * frequency),1) ;
% 
%     for k = frontN+1:Nframes-laterN-1
%         IsOnes_kSegment = IsOnes( k-frontN:k+laterN ) ;
%         IsContinuousOnes(k) = JudgeContinuousOnes_One( IsOnes_kSegment,frontN+1,minRateFront,minRateLater );
%     end
% 
%     %% 输入第k时刻邻域的 IsOnes_kSegment（是否为1） 判断 该点是否为连续 =1
%     % 待判断的点为 IsOnes_kSegment 的第k个点
%     function IsContinuousOnes_k = JudgeContinuousOnes_One( IsOnes_kSegment,k,minRateFront,minRateLater )
% 
%     IsContinuousOnes_k = 0;
%     N = length(IsOnes_kSegment);
%     if IsOnes_kSegment(k)==1 
%         if IsOnes_kSegment(k-1)==1 || IsOnes_kSegment(k+1) == 1  % 前后紧邻至少有一个满足
%             IsOnes_Front = IsOnes_kSegment( 1:k-1 );
%             IsOnes_Later = IsOnes_kSegment( k+1:N );
%             if sum( IsOnes_Front ) >= ceil( minRateFront*(k-1) )       % 前面1的个数满足
%                if  sum( IsOnes_Later ) >= fix( minRateLater*(N-k) )   % 后面1的个数满足
%                    IsContinuousOnes_k = 1 ;               
%                end
%             end
%         end    
%     end
