%% 2015.4.23


 %% 平滑（可改进为实时运行）
 % 目的：将由于噪声导致一段静止时间段内JudgeData的不连续区间段补平
 % 思路：1） 以时间段为单位进行平滑，专门针对这样的区间段进行平滑：1>前后均被 超过stepN/4 个1夹着的0时间段  2>0时间段时间长度小于 stepN
 %       2）当这个时间段满足某个指标时，认为这个时间段内均为 1，否则均不进行处理
 % 指标：设待判定区间段长度为 M ，满足一下条件则认为是静止状态，均置为1
 %  1）区间前 M 个为1
 %  2）区间后M个为1
 %  3）满足1）和2）则认为OK，结束；不按满足则进入4）判定
 %  4）（区间前2M+区间+区间后M）中有超过 70% 为1  
 %  5)  区间后M超过50%为1
 %  满足4）和5） 则认为 该区间 均为1
 %%
 % JudgeData： [1*N]  满足为1，不满足为0
 % stepN :最长平滑窗口
function JudgeData = SmoothJudgeData( JudgeData,stepN,SmoothRate )
Nframes = length(JudgeData) ;
stepN = max(stepN,4);
clipN = max(fix(stepN/6),2);     % 有效区间判定左右最小连续1的个数

if SmoothRate>0.75
    SmoothRate = 0.74;  % 最大0.75
elseif SmoothRate<0.6
    SmoothRate = 0.6;
end

fillNum = 0;
%% 实时搜索待平滑的区间 NeedSmoothWin

winStart = 0;
for k=clipN+1:Nframes-clipN+1
    % 记录所有的0连续区间，当区间长度<=stepN时进行详细判定
    if JudgeData(k)==0
        if JudgeData(k-1) == 1
            if sum(JudgeData(k-clipN:k-1))==clipN   % 前 clipN 个均为1
                %% 1...1->0 
                winStart = k;   % 记录0区间的起始位
            end
        end
    else
        if JudgeData(k-1) == 0
            if sum(JudgeData(k:k+clipN-1))==clipN   % 后 clipN 个均为1
                %% 0...0->1 
                if winStart==0 
                    continue;       % 第一次先找到 0...0->1 丢弃
                end
                winEnd = k-1;   % 记录0区间的结束位
                M = winEnd-winStart+1 ;
                if M < stepN                
                   %% 捕获到一个有效的待平滑区间 
                   IsDoSmooth = 0;
                    % 1）区间前 M 个为1
                    if sum( JudgeData(winStart-M:winStart-1) )==M && sum( JudgeData(winEnd+1:winEnd+M) )==M
                       %%% 判断这个区间OK：均置为1  (优)
                       IsDoSmooth = 1;
                    elseif winStart-M*2+1>1 && winEnd+M<Nframes
                        if sum( JudgeData(winStart-M*2+1:winEnd+M) ) >= M*3*SmoothRate && sum( JudgeData(winEnd+1:winEnd+M) )>=M*0.5
                            %%% 判断这个区间OK：均置为1  (次优)
                            IsDoSmooth = 1;
                        end
                    end
                    %%% 判断这个区间OK：均置为1
                    if IsDoSmooth == 1
                        JudgeData(winStart:winEnd) = 1 ;
                        fillNum = fillNum + M;
                    end
                end
            end
        end
    end
end


% display( sprintf('fillNum = %0.0f',fillNum) );
