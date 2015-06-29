%% xyz

%% 马克点连续性判断
% dPi_ConJudge ： 连续性判断指标的大小：前后两帧的位移模
% ContinuesFlag = 0   不连续
%               =1    连续，且是与跟踪成功马克点连续
%               =2   连续，和跟踪失败的点连续
function [ otherMakers_k,dPi_ConJudgeOut ] = ContinuesJudge( otherMakers_k,otherMakers_k_last,trackedMakerPosition,...
    k_vision,makerTrackThreshold )


% 前一时刻跟踪成功时，给出当前点是否相对跟踪成功的点连续。  Continues = 1
% 前一时刻跟踪失败时，给出当前每个点是否为连续点的结果,且记录着个点能连续到往前最早（但不超过dT）的点的位置和时间。
%        Continues = 2 ，

% global inertialFre visionFre  moveDistance
% 记录每个马克点的连续特性
MaxOtherMarkerBufN = size(otherMakers_k.ContinuesFlag,2);
M = otherMakers_k.otherMakersN ;
otherMakers_k.ContinuesFlag = zeros(1,MaxOtherMarkerBufN) ; % 不连续
otherMakers_k.ContinuesLastPosition = NaN(3,MaxOtherMarkerBufN) ;
otherMakers_k.ContinuesLastTime = NaN(1,MaxOtherMarkerBufN) ;
otherMakersPosition_k = otherMakers_k.Position ;  

dPi_ConJudgeOut=nan;
dPi_ConJudge = nan(1,M);

if k_vision>1 
    trackedMakerPosition_kLast = trackedMakerPosition(:,k_vision-1) ;
    last_Tracked_i =  GetTrackedi( trackedMakerPosition_kLast,otherMakers_k_last );  % 上一时刻跟踪成功的点的序号
    
%     if ~isnan(trackedMakerPosition(1,k_vision-1))
%         %% 只判断当前马克点是否与前时刻跟踪成功的马克点连续
%         trackedMakerPosition_kLast = trackedMakerPosition(:,k_vision-1) ;
%         otherMakersPosition_k_Dis = otherMakersPosition_k(:,1:M)-repmat(trackedMakerPosition_kLast,1,M) ;
%         otherMakersPosition_k_Dis_Norm = zeros(1,M);
%         for j=1:M
%             otherMakersPosition_k_Dis_Norm(j) = normest(otherMakersPosition_k_Dis(:,j));
%         end
%         [dPi_ConJudge,minCon_k] = min(otherMakersPosition_k_Dis_Norm);  
%         if dPi_ConJudge < makerTrackThreshold.MaxContinuesDisplacement
%     %         trackedMakerPosition_k_OK = otherMakersPosition_k(:,m) ;
%     %         TrackFlag = 1;
%     %         fprintf('马克点连续：位移=%0.4f，跟踪OK \n',Min_otherMakersPosition_k_Dis_Norm);
%             
%             otherMakers_k.ContinuesFlag(minCon_k) = 1 ; % 连续，且是与跟踪成功马克点连续
%             otherMakers_k.ContinuesLastPosition(:,minCon_k) = trackedMakerPosition_kLast ;
%             otherMakers_k.ContinuesLastTime(minCon_k) = otherMakers_k_last.time ;
%             otherMakers_k.ContinuesLastK(minCon_k) = k_vision-1 ;
%             otherMakers_k.ContinuesLasti(minCon_k) = lastTrackedi;  % 记录紧邻的上一时刻马克点序号
%         else
%             
%         end
%         
%     else
        %% 判断当前马克点是否为连续马克点，记录每个点对应的最早（但不超过dT）连续点
        M_last = otherMakers_k_last.otherMakersN ;
        if M_last==0
            % 上时刻无马克点
            for i=1:M
                otherMakers_k.ContinuesFlag(i) = 0 ; % 不连续
                otherMakers_k.ContinuesLastPosition(:,i) = NaN ;
                otherMakers_k.ContinuesLastTime(i) = NaN ;
                otherMakers_k.ContinuesLastK(i) = NaN ;
                otherMakers_k.ContinuesLasti(i) = NaN;  % 记录紧邻的上一时刻马克点序号
            end
            dPi_ConJudgeOut=nan;
            return;
        end
        % 一共有 M*M_last 种组合
        for i=1:M
            % 当期第i个点 与 上一时刻每个点相减
            dPi = repmat(otherMakers_k.Position( :,i ),1,M_last)- otherMakers_k_last.Position(:,1:M_last) ;
            dPiNorm = zeros(1,M_last);
            for j=1:M_last
                dPiNorm(j) = normest(dPi(:,j));
            end
            
            [dPi_ConJudge(i),min_last_i] = min(dPiNorm);     % min_last_i 为当前点对应连续的上一时刻点的序号
            if normest(dPi_ConJudge(i)) < makerTrackThreshold.MaxContinuesDisplacement
                %  otherMakers_k.Position( :,i ) 与 otherMakers_k_last.Position(:,min_last_i) 连续
                % 找到一个连续的点，记录上一点
                if ~isnan(last_Tracked_i) && last_Tracked_i==min_last_i
                    otherMakers_k.ContinuesFlag(i) = 1 ; % 和跟踪成功的点连续
                else
                    otherMakers_k.ContinuesFlag(i) = 2 ; % 和跟踪失败的点连续
                end
                
                otherMakers_k.ContinuesLasti(i) = min_last_i;  % 记录紧邻的上一时刻马克点序号
                % 如果前一个点为连续点，则将前一个点的连续记录传递过来
                if otherMakers_k_last.ContinuesFlag(min_last_i) == 2
                    
                    otherMakers_k.ContinuesLastK(i) = otherMakers_k_last.ContinuesLastK(min_last_i) ; % 传递记录上一个时刻存储的连续信息
                    otherMakers_k.ContinuesLastPosition(:,i) = otherMakers_k_last.ContinuesLastPosition(:,min_last_i) ;
                    otherMakers_k.ContinuesLastTime(i) = otherMakers_k_last.ContinuesLastTime(min_last_i);
                elseif otherMakers_k_last.ContinuesFlag(min_last_i) == 0 || isnan(otherMakers_k_last.ContinuesFlag(min_last_i))
                    otherMakers_k.ContinuesLastK(i) = k_vision-1 ; % 直接记录上一个时刻
                    otherMakers_k.ContinuesLastPosition(:,i) = otherMakers_k_last.Position( :,min_last_i ) ;
                    otherMakers_k.ContinuesLastTime(i) = otherMakers_k_last.time ;                    
                elseif otherMakers_k_last.ContinuesFlag(min_last_i) == 1 
                    % 与跟踪成功点连续检测成功，但跟踪识别失败的情况，传递到现在。如果传递时间超过2秒，则不再传递。
                    if (otherMakers_k_last.ContinuesLastTime(min_last_i)-otherMakers_k.time) > 20
                        otherMakers_k.ContinuesFlag(i) = 2 ;
                    else
                        otherMakers_k.ContinuesFlag(i) = 1 ;
                    end                    
                    otherMakers_k.ContinuesLastK(i) = otherMakers_k_last.ContinuesLastK(min_last_i) ; % 传递记录上一个时刻存储的连续信息
                    otherMakers_k.ContinuesLastPosition(:,i) = otherMakers_k_last.ContinuesLastPosition(:,min_last_i) ;
                    otherMakers_k.ContinuesLastTime(i) = otherMakers_k_last.ContinuesLastTime(min_last_i);
                end
            else
                otherMakers_k.ContinuesFlag(i) = 0 ; % 不连续
                otherMakers_k.ContinuesLastPosition(:,i) = NaN ;
                otherMakers_k.ContinuesLastTime(i) = NaN ;
                otherMakers_k.ContinuesLastK(i) = NaN ;
                otherMakers_k.ContinuesLasti(i) = NaN;  % 记录紧邻的上一时刻马克点序号
            end
            % 如果之前已经有点对应着同一点，则保留最近的点的判断，其他给 NaN
            for j=1:i-1
               if  otherMakers_k.ContinuesLasti(j) == otherMakers_k.ContinuesLasti(i)
                   if dPi_ConJudge(i) < dPi_ConJudge(j)
                       % 当前的点距离上时刻该点更近
                       otherMakers_k.ContinuesFlag(j) = 0;
                       otherMakers_k.ContinuesLastPosition(:,j) = NaN ;
                       otherMakers_k.ContinuesLastTime(j) = NaN ;
                       otherMakers_k.ContinuesLastK(j) = NaN ;
                       otherMakers_k.ContinuesLasti(j) = NaN;
                   else
                       otherMakers_k.ContinuesFlag(i) = 0;
                       otherMakers_k.ContinuesLastPosition(:,i) = NaN ;
                       otherMakers_k.ContinuesLastTime(i) = NaN ;
                       otherMakers_k.ContinuesLastK(i) = NaN ;
                       otherMakers_k.ContinuesLasti(i) = NaN;
                   end
               end
            end
        end
        %%  记录 dPi_ConJudge 的最小值
        if M>0
            if dPi_ConJudge(i)<dPi_ConJudgeOut || isnan(dPi_ConJudgeOut)
                dPi_ConJudgeOut = dPi_ConJudge(i);
            end
        else
            dPi_ConJudgeOut = NaN;
        end
%     end
else
    dPi_ConJudge=nan;
end

% 根据 trackedMakerPosition_k 计算跟踪成功的马克点对应的序号（很不好的办法，但由于不想破坏之前的数据结构，暂时这么做）
function Tracked_i =  GetTrackedi( trackedMakerPosition_k,otherMakers_k )
if isnan(trackedMakerPosition_k(1))
    Tracked_i = NaN;
    return;
end
M = otherMakers_k.otherMakersN ;
dPi = repmat(trackedMakerPosition_k,1,M)- otherMakers_k.Position(:,1:M) ;
            dPiNorm = zeros(1,M);
for j=1:M
    dPiNorm(j) = normest(dPi(:,j));
end

[dPi_ConJudge,min_last_i] = min(dPiNorm);   
Tracked_i = min_last_i;