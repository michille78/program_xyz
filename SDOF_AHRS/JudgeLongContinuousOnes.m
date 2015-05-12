%% xyz 2015.4.30

%% 判断长时间0加速度区间
%%% 作用：
% 1）初始零位时段 = 第一个 长时间0加速度区间
% 2）转轴优化时需要寻找 转角最小的替代零位，需要满足  长时间0加速度 条件


%% 判断某个时刻的 IsOnes=1 是否长时间保持
% IsOnes ： [1*N]
% 动态的是否0加速度判断结果，利用这个判断初始静止状态持续的时间长度。使用前先做一个较长时间（IsLongContinuousOnes_SmoothStepTime）的平滑
% 当出现连续 IsLongContinuousOnes_JudgeStepTime 长时间0加速度时判断为静止状态开始
% 当出现 非 连续 IsLongContinuousOnes_JudgeStepTime 长时间保持静止判断为静止状态结束
function [ LongContinuousOnsStart,LongContinuousOnsEnd,IsLongContinuous ] = JudgeLongContinuousOnes...
    ( IsOnes,IsLongContinuousOnes_SmoothStepTime,IsLongContinuousOnes_JudgeStepTime,frequency ) 
JudgeRate = 0.8 ;
IsLongContinuousOnes_AbandonTime = IsLongContinuousOnes_JudgeStepTime/6 ; % 丢弃每个区间前后一小段的时间

SmoothStepN = fix( IsLongContinuousOnes_SmoothStepTime*frequency ) ;
IsOnesSmooth = SmoothJudgeData( IsOnes,SmoothStepN,0.7 ) ;  % 进行较大的步长平滑后再做判断

stepN = fix( IsLongContinuousOnes_JudgeStepTime*frequency ) ;  %  某个区间段满足 stepN 长数据连续为1 ， 则该区间均为1， 否则为0
abandonNum = fix(IsLongContinuousOnes_AbandonTime*frequency);

Nframes = length(IsOnesSmooth) ;
k=1;

LongContinuousOnsStart = zeros(1,10);
LongContinuousOnsEnd = zeros(1,10);
LongContinuousOnes_k = 0;

while k<Nframes-stepN
%     disp('JudgeLongContinuousOnes...')
    %% find LongContinuousOnsStart
    findStartOK = 0;
    findEndOK = 0;
    while k < Nframes-stepN
        if IsOnesSmooth(k)==1
            sum_k = sum( IsOnesSmooth(k:k+stepN-1) );
            if sum_k >= stepN*JudgeRate      % 往后的 stepN 个中，为1的个数足够多 。 认定改为是1个区间的开始
                                LongContinuousOnsStart_k = k  ;        % 出现连续 IsLongContinuousOnes_JudgeStepTime 长时间0加速度时判断为静止状态开始
                findStartOK = 1 ;
                break;
            end        
        end
        k = k+1 ;
    end
    if findStartOK==1
        %% find initialStaticStop
        k = k+stepN-1 ;
        while k<Nframes
                sum_k = sum( IsOnesSmooth(k-stepN+1:k) );
                if sum_k < stepN*JudgeRate  % 往前的 stepN 个中，为1的个数不够多 。 认定改为是1个区间的结束
                    % 从第k个开始已经不是连续 IsLongContinuousOnes_JudgeStepTime 长时间保持0加速度                
                    for i=1:stepN
                        if IsOnesSmooth( k-i+1 )==1
                            LongContinuousOnsEnd_k = k-i+1  ; % 取这个区间内最后一个为1的作为结束点
                            findEndOK = 1 ;
                            break;
                        end
                    end                  
                    break;
                end    
            k = k+1 ;
        end        
    end
    if findStartOK==1 && findEndOK==1
        LongContinuousOnes_k = LongContinuousOnes_k+1 ;
       LongContinuousOnsStart(LongContinuousOnes_k)  = LongContinuousOnsStart_k ;
       LongContinuousOnsEnd(LongContinuousOnes_k) = LongContinuousOnsEnd_k ;
       if LongContinuousOnsEnd(LongContinuousOnes_k)-LongContinuousOnsStart(LongContinuousOnes_k) > abandonNum*5
            LongContinuousOnsStart(LongContinuousOnes_k) = LongContinuousOnsStart(LongContinuousOnes_k)+abandonNum ;
            LongContinuousOnsEnd(LongContinuousOnes_k) = LongContinuousOnsEnd(LongContinuousOnes_k)-abandonNum ;
        end
    end
end

LongContinuousOnsStart = LongContinuousOnsStart(1:LongContinuousOnes_k) ;
LongContinuousOnsEnd = LongContinuousOnsEnd(1:LongContinuousOnes_k) ;

%% IsLongContinuous
IsLongContinuous = zeros( 1,Nframes );
LongContinuesTime = zeros( 1,Nframes );
for k=1:LongContinuousOnes_k
    temp = LongContinuousOnsStart(k):LongContinuousOnsEnd(k) ;
    IsLongContinuous( temp ) = 1 ;
    LongContinuesTime( LongContinuousOnsStart(k) ) = ( LongContinuousOnsEnd(k)-LongContinuousOnsStart(k)+1 )/frequency ;
end

% figure('name','JudgeLongContinuousOnes')
% axes('YLim',[-0.2 1.2])
% hold on
% plot(IsOnes,'k.')
% plot(IsOnesSmooth*0.97,'b.')
% plot(IsLongContinuous*0.95,'ro')
% 
% legend('IsOnes','IsOnesSmooth','IsLongContinuous')

