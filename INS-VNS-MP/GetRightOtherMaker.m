%% xyz 2015.5.25
%% otherMakers 中马克点识别：在多个马克点中找到正确的马克点
% 马克点丢失时给出 NaN

%% 判断思路：比较2个相对位移矢量：1）dT(3 sec)运动时间时  2）dS（1m）运动位移长度时
% 1)dT(3 sec)时间内，惯性和视觉位移向量的大小差<0.1m，方向差<60°（当位移矢量长度小于0.2m时不比较方向）

function trackedMakerPosition = GetRightOtherMaker( otherMakers,InertialData )
global inertialFre visionFre makerTrackThreshold
%% 阈值参数设置
dT_1 = 3 ;    % sec 轨迹连续判断时间步长
dS_2 = 1 ;    % m   轨迹连续判断位移步长
makerTrackThreshold.MaxPositionError_dT = dT_1*0.05 ;   % 固定时间的最大运动距离误差：0.05m每s
makerTrackThreshold.MaxPositionError_dS = dS_2*0.2;     % 运动固定距离位移的最大运动距离误差：运动距离的20%
makerTrackThreshold.MaxDisplaceAngle = 45*pi/180 ;      % 运动固定距离位移的最大位移方向角度差
%% load data
visionFre = otherMakers(1).frequency ;
MarkerSet= otherMakers(1).MarkerSet ;
inertialTime = InertialData.time ;
inertialFre = InertialData.frequency ;

switch MarkerSet 
    case 'Head'
        InertialPosition = InertialData.HeadPosition ;
    case 'Hip'
        InertialPosition = InertialData.HipPosition ;
end

dT_Ninertial = fix(dT_1/inertialFre) ;
dT_Nvision = fix(dT_1/visionFre) ;

%%
makerN = length(otherMakers);
trackedMakerPosition = NaN(3,makerN); % 判断成功的马克点位置

wh = waitbar(0,'SearchDistanceK');
for k=1:makerN
    maker_time = otherMakers(k).time ;
    % inertial_k
    inertialTimeErr = abs( inertialTime - maker_time );
    [~,inertial_k] = min(inertialTimeErr);
    %  last_dT_k
    inertial_dT_k_last = inertial_k - dT_Ninertial ;
    inertial_dT_k_last = max(inertial_dT_k_last,1);
    vision_dT_k_last = k - dT_Nvision ;
    vision_dT_k_last = max(vision_dT_k_last,k);
    
    % find the point which moved dS_2
    dS_Inertial_last_k = SearchDistanceK( InertialPosition,inertial_k,dS_2,trackedMakerPosition ) ;    
    
    otherMakers_k = otherMakers(k) ;    
    
%      dbstop in JudgeMaker
    trackedMakerPosition(:,k) = JudgeMaker...
        ( otherMakers_k,inertial_k,trackedMakerPosition,InertialPosition,inertial_dT_k_last,...
            vision_dT_k_last,dS_Inertial_last_k ) ;
    waitbar(k/makerN);
end
close(wh);

%% Judge which is the right maker
% 1) 固定运动时间位移判断：只判断位移差长度
% 2）固定运动距离位移判断：同时判断位移差长度和方向

function trackedMakerPosition_k = JudgeMaker...
( otherMakers_k,inertial_k,trackedMakerPosition,InertialPosition,inertial_dT_k_last,...
            vision_dT_k_last,dS_Inertial_last_k )
global inertialFre visionFre makerTrackThreshold

trackedMakerPosition_k = NaN;  % 跟踪马克点失败置 NaN

M = otherMakers_k.otherMakersN ;
otherMakersPosition_k = otherMakers_k.Position ;

for i=1:M
    otherMakersPosition_k_i = otherMakersPosition_k(:,i);
    
    %% dT 时间段的位移差：只考虑位移矢量大小
    % 如果 vision_dT_k_last 没跟踪成功，把时间往前推
    while vision_dT_k_last>0 && isnan( trackedMakerPosition(1,vision_dT_k_last) ) % trackedMakerPosition(1) 必须已知（不为nan）
        vision_dT_k_last = vision_dT_k_last-1 ;
        inertial_dT_k_last = fix( vision_dT_k_last*inertialFre/visionFre );
        inertial_dT_k_last = max(inertial_dT_k_last,1);
    end
    if vision_dT_k_last==0
        % 第一个马克点还未判断，利用惯性和视觉的坐标系原点重合
        trackedMakerPosition_last_k_dT = [0;0;otherMakersPosition_k(3,1)] ;
    else
        trackedMakerPosition_last_k_dT = trackedMakerPosition(:,vision_dT_k_last) ;
    end
    
    dP_Inertial = InertialPosition(:,inertial_k) - InertialPosition(:,inertial_dT_k_last);
    dP_Vision = otherMakersPosition_k_i - trackedMakerPosition_last_k_dT;
    dPError_dT = dP_Inertial-dP_Vision ;
    if normest(dPError_dT) > makerTrackThreshold.MaxPositionError_dT
        % 不满足，对这个马克点不需要进行进一步判断
        continue;
    end
    %% dS 位移长度段的位移差：同时考虑位移差大小和方向
    if isnan(dS_Inertial_last_k)
        % 找不到运动大于 dS 长度且trackedMakerPosition中跟踪到的点
        % 只能认为也满足（但可信度没有那么高！！！）
        trackedMakerPosition_k = otherMakersPosition_k_i ;
        continue; 
    end
    dS_Vision_last_k = fix(dS_Inertial_last_k*inertialFre/visionFre) ;    
    dP_Inertial = InertialPosition(:,inertial_k) - InertialPosition(:,dS_Inertial_last_k);
    dP_Vision = otherMakersPosition_k_i - trackedMakerPosition(:,dS_Vision_last_k);
    dPError_dS = dP_Inertial-dP_Vision ;
    if normest(dPError_dS) < makerTrackThreshold.MaxPositionError_dS
        % 距离满足，判断方向
        temp = dP_Inertial.*dP_Vision / normest(dP_Inertial) / normest(dP_Vision) ;
        angleErr = acos(temp) ;
        if angleErr < makerTrackThreshold.MaxDisplaceAngle
           % 满足所有条件，跟踪马克点成功
           trackedMakerPosition_k = otherMakersPosition_k_i ;
           break;
        end
    end
end
if isnan(trackedMakerPosition_k)
   disp('tracke MakerPosition failed') 
end

%% 查找惯性k时刻前运动了大于 dS 距离的最近的点
% 且该点trackedMakerPosition跟踪成功
function dS_Inertial_last_k = SearchDistanceK( InertialPosition,kCurrent,dS,trackedMakerPosition )
InertialPosition_kSea = InertialPosition(:,kCurrent);
dS_Inertial_last_k = NaN ;     % 默认设置：寻找失败
%wh = waitbar(0,'SearchDistanceK');
for i=1:kCurrent-1
    dP = InertialPosition(:,kCurrent-i) - InertialPosition_kSea ;
    distance = normest( dP ); 
    if distance > dS && ~isnan(trackedMakerPosition(kCurrent-i))
        % 距离满足，且 trackedMakerPosition 跟踪成功
        dS_Inertial_last_k = kCurrent-i ;
        break;
    end
 %   waitbar(i/kCurrent);
end
%close(wh)

