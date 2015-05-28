%% xyz 2015.5.25
%% otherMakers 中马克点识别：在多个马克点中找到正确的马克点
% 马克点丢失时给出 trackedMakerPosition = NaN
%%% 单马克点跟踪判定思路

%%%  第一个点之后跟踪方法
% 1）检验高度，高度满足的话，进入2）  [错！！舍弃这一步！高度方向不可用。因为惯性采用Hip作为原点，只有弯腰会导致高度变化，身体整体下蹲不会发生高度变化。]
%  注：后续如果根据脚的高度，计算出头相对地面的高度，可用高度信息大大提高检测精度！！！
% 2） 马克点连续性判定
%   如果上一时刻的单马克点跟踪成功，则在当前马克点中寻找与上一时刻最接近的马克点，位移差模小于 1/visionFre*0.5（最小2cm）
%   时认为数据连续，跟踪成功。( TrackFlag=1 )
%       判定失败，马克点不连续，进入第二步。
% 3) 短时间（dT=3）运动的过程，惯性和视觉位移差 dPError_dT 进行判定， dPError_dT(i) = normest(dP_Inertial-dP_Vision)
%       3.1） dP_Inertial模小于0.03 m，处于准静止状态。若dPError_dT模大于 0.01*moveTime 则跟踪失败。
%             若连续则OK ( TrackFlag=3.1 )    否则 ( TrackFlag=1.31 )  进入4） 
%       3.2） dP_Inertial模大于0.03 m，处于运动状态。 
%             dPError_dT 的模小于 0.01*moveTime 时跟踪OK ( TrackFlag=3.5 )
%             若dP_Vision的模非常小，直接剔除。    否则 ( TrackFlag=1.35 ) 进入4）
%       1) 2)判定均失败，马克点不连续，且惯性和视觉位移误差略大，进入第判定3）。（可能打算在快速运动的情况中）
% 4) 寻找惯性运动 dS 长度(大于 moveDistance)，且马克点跟踪成功的是时刻，判定 dP_Inertial 和 dP_Vision：
%       4.0) 搜索这类点失败： TrackFlag=-1.29
%       4.1） dPError_dS 的模小于 MaxPositionError_dS（moveDistance*0.35） 
%       4.2) dP_Inertial 和 dP_Vision 的夹角小于MaxDisplaceAngle
%       同时满足 4.1） 和 4.2）认为马克点跟踪成功 ( TrackFlag=4+TrackFlag )，否则 ( TrackFlag=-TrackFlag )
%%% 第一个点的跟踪方法
% 通过运动状态搜索第一个点

%% 判断思路：比较2个相对位移矢量：1）dT(3 sec)运动时间时  2）dS（1m）运动位移长度时
% 1)dT(3 sec)时间内，惯性和视觉位移向量的大小差<0.1m，方向差<60°（当位移矢量长度小于0.2m时不比较方向）

function trackedMakerPosition = GetRightOtherMaker( otherMakers,InertialData )
global  makerTrackThreshold moveDistance
global otherMakersTime  inertialTime 
global visionFre  inertialFre
%% load data
visionFre = otherMakers(1).frequency ;
MarkerSet= otherMakers(1).MarkerSet ;
inertialTime = InertialData.time ;
inertialFre = InertialData.frequency ;

%% 阈值参数设置
moveTime = 2 ;          % sec 轨迹连续判断时间步长
moveDistance = 0.4 ;      % m   轨迹连续判断位移步长  （经验值建议0.4m-0.7m）
makerTrackThreshold.MaxContinuesDisplacement = min( 1/visionFre*0.3,0.02) ; % 马克点连续判断最大位移模
makerTrackThreshold.PositionErrorBear_dT = 0.01*moveTime;   % 固定时间的最大运动距离误差（第一步）：位移差在这个范围内的，直接判定<校验1>通过
makerTrackThreshold.MaxStaticDisp_dT = max(0.005*moveTime,0.02) ;           % 固定时间的最大运动距离误差（第二步）：（第一步不通过后）位移差的长度是惯性位移长度的MaxPositionError_dT倍以内
makerTrackThreshold.MaxPositionError_dS = moveDistance*0.7;     % 运动固定距离位移的最大运动距离误差：运动距离的50% （主要是依赖角度约束）
makerTrackThreshold.MaxDisplaceAngle = 20*pi/180 ;      % 运动固定距离位移的最大位移方向角度差


switch MarkerSet 
    case 'Head'
        InertialPosition = InertialData.HeadPosition ;
    case 'Hip'
        InertialPosition = InertialData.HipPosition ;
end

dT_Ninertial = fix(moveTime/inertialFre) ;

%%
MarkerTN = length(otherMakers);
trackedMakerPosition = NaN(3,MarkerTN); % 判断成功的马克点位置
%% 先要求第一个点已知 
% trackedMakerPosition(:,1)  = otherMakers(1).Position(:,1) ;

% otherMakersNew = struct;
TrackFlag = zeros(1,MarkerTN);
ObjectMakerHigh = CalObjectMakerHigh( otherMakers );

otherMakers(1).ContinuesFlag = 0 ; % 不连续
otherMakers(1).ContinuesLastPosition = NaN ;
otherMakers(1).ContinuesLastTime = NaN ;
otherMakers(1). ContinuesLastK = NaN;


dPi_ConJudge = NaN(1,MarkerTN);   
dPError_dT_xy = NaN(1,MarkerTN);  
dPError_dT_z = NaN(1,MarkerTN);  
dPError_dS_xyNorm = NaN(1,MarkerTN);  
dP_Inertial_xyNorm = NaN(1,MarkerTN);  
angleErr_dS = NaN(1,MarkerTN);  
dP_Inertial_z = NaN(1,MarkerTN); 

 wh = waitbar(0,'SearchDistanceK');
for k=1:MarkerTN
    inertial_k = VisionK_to_InertialK(k);
    
    %  last_dT_k
    inertial_dT_k_last = inertial_k - dT_Ninertial ;
    inertial_dT_k_last = max(inertial_dT_k_last,1);
    vision_dT_k_last = InertialK_to_VisionK(inertial_dT_k_last);           
    
    otherMakers_k = otherMakers(k) ;  
    if k>1
        otherMakers_k_last = otherMakers(k-1) ; 
    else
        otherMakers_k_last = [];
    end
%      dbstop in JudgeMaker
   if k==150
      disp('') 
   end
     
    [ trackedMakerPosition(:,k),otherMakersNew_k,TrackFlag(k),JudgeIndex  ] = JudgeMaker...
        ( otherMakers_k,otherMakers_k_last,k,inertial_k,trackedMakerPosition,InertialPosition,inertial_dT_k_last,...
            vision_dT_k_last,ObjectMakerHigh ) ;
    otherMakers(k)=otherMakersNew_k;
    dPi_ConJudge(k) = JudgeIndex.dPi_ConJudge  ;
    dPError_dT_xy(k) = JudgeIndex.dPError_dT_xy ;
    dPError_dT_z(k) = JudgeIndex.dPError_dT_z ;
    dPError_dS_xyNorm(k) = JudgeIndex.dPError_dS_xyNorm ;
    dP_Inertial_xyNorm(k) = JudgeIndex.dP_Inertial_xyNorm ;
    dP_Inertial_z(k) = JudgeIndex.dP_Inertial_z ;
    angleErr_dS(k) = JudgeIndex.angleErr_dS ;
    
    if mod(k,fix(MarkerTN/10))==0
        waitbar(k/MarkerTN);
    end
end
 close(wh);

figure('name','trackFlag')
plot(otherMakersTime,TrackFlag,'.')
xlabel('time sec')

figure('name','dPi_ConJudge')
plot(otherMakersTime,dPi_ConJudge)
temp = makerTrackThreshold.MaxContinuesDisplacement ;
line( [otherMakersTime(1) otherMakersTime(MarkerTN)],[temp temp],'color','r' )

xlabel('time sec')

figure('name','dPError_dT_xy')
plot(otherMakersTime,dPError_dT_xy)
temp = makerTrackThreshold.PositionErrorBear_dT ;
line( [otherMakersTime(1) otherMakersTime(MarkerTN)],[temp temp],'color','r' )
xlabel('time sec')
legend( 'dPError\_dT\_xy','PositionErrorBear\_dT' )

figure('name','dPError_dT_z')
plot(otherMakersTime,dPError_dT_z)
temp = makerTrackThreshold.PositionErrorBear_dT ;
line( [otherMakersTime(1) otherMakersTime(MarkerTN)],[temp temp],'color','r' )
xlabel('time sec')
legend( 'dPError\_dT\_z','PositionErrorBear\_dT' )

figure('name','dP_Inertial_xyNorm')
plot(otherMakersTime,dP_Inertial_xyNorm)
temp = makerTrackThreshold.MaxStaticDisp_dT ;
line( [otherMakersTime(1) otherMakersTime(MarkerTN)],[temp temp],'color','r' )
xlabel('time sec')
legend( 'dPError\_dT','MaxStaticDisp\_dT' )

figure('name','dP_Inertial_z')
plot(otherMakersTime,dP_Inertial_z)
temp = makerTrackThreshold.MaxStaticDisp_dT ;
line( [otherMakersTime(1) otherMakersTime(MarkerTN)],[temp temp],'color','r' )
xlabel('time sec')
legend( 'dPError\_dT','MaxStaticDisp\_dT' )


figure('name','dPError_dS_xyNorm')
plot(otherMakersTime,dPError_dS_xyNorm)
temp = makerTrackThreshold.MaxPositionError_dS ;
line( [otherMakersTime(1) otherMakersTime(MarkerTN)],[temp temp],'color','r' )
xlabel('time sec')

figure('name','angleErr_dS')
plot(otherMakersTime,angleErr_dS*180/pi)
temp = makerTrackThreshold.MaxDisplaceAngle*180/pi ;
line( [otherMakersTime(1) otherMakersTime(MarkerTN)],[temp temp],'color','r' )
xlabel('time sec')


FailTrackFlagNum = sum( TrackFlag<=0 );
fprintf( 'FailTrackFlagNum=%d ( %0.3f ) \n',FailTrackFlagNum,FailTrackFlagNum/length(TrackFlag) );

% 计算马克点的高度 ObjectMakerHigh ：
% 认为初始时刻0.5秒内目标马克点是一定跟踪到的

function ObjectMakerHigh = CalObjectMakerHigh( otherMakers )
global  visionFre
N = fix(visionFre*0.5);
for k=1:N
    if otherMakers(k).otherMakersN > 0 
        otherMakersPosition_k = otherMakers(k).Position ;
        if abs(otherMakersPosition_k(1))<0.1 && abs(otherMakersPosition_k(2))<0.1
            ObjectMakerHigh = -otherMakersPosition_k(3) ;
            return;
        end
    end
end


%% Judge which is the right maker
% 1) 固定运动时间位移判断：只判断位移差长度
% 2）固定运动距离位移判断：同时判断位移差长度和方向

function [ trackedMakerPosition_k_OK,otherMakers_k,TrackFlag,JudgeIndex ] = JudgeMaker...
( otherMakers_k,otherMakers_k_last,k_vision,inertial_k,trackedMakerPosition,InertialPosition,inertial_dT_k_last,...
            vision_dT_k_last,ObjectMakerHigh )
global inertialFre visionFre makerTrackThreshold moveDistance

JudgeIndex.dPi_ConJudge = NaN ;
JudgeIndex.dPError_dT_xy = NaN ;
JudgeIndex.dPError_dT_z = NaN ;
JudgeIndex.dPError_dS_xyNorm = NaN ;
JudgeIndex.dP_Inertial_xyNorm = NaN ;
JudgeIndex.angleErr_dS = NaN  ;
JudgeIndex.dP_Inertial_z = NaN  ;

trackedMakerPosition_k_OK = NaN;  % 跟踪马克点失败置 NaN
TrackFlag = 0;

M = otherMakers_k.otherMakersN ;
otherMakersPosition_k = otherMakers_k.Position ;

if isempty(otherMakersPosition_k)
    return;
end

%% 马克点连续性判断
[ otherMakers_k,dPi_ConJudge ] = ContinuesJudge( otherMakers_k,otherMakers_k_last,trackedMakerPosition,k_vision );
JudgeIndex.dPi_ConJudge = dPi_ConJudge ;

%% dT 时间段的位移差：只考虑位移矢量大小
% 如果 vision_dT_k_last 没跟踪成功，把时间往前推
while vision_dT_k_last>1 && isnan( trackedMakerPosition(1,vision_dT_k_last) ) % trackedMakerPosition(1) 必须已知（不为nan）
    vision_dT_k_last = vision_dT_k_last-1 ;
    inertial_dT_k_last = VisionK_to_InertialK(vision_dT_k_last);
end
if isnan( trackedMakerPosition(1,vision_dT_k_last) )
    %% 寻找第一个点（第一个点：怎么都找不到之前跟踪成功的点）
    % 用其他连续点计算，分析当前马克点中每一个连续点，如果某个连续点的 dT 判断通过，则认为这是第一个点
    IsSearchingFirst = 0;
    for i=1:M
        if  otherMakers_k.ContinuesFlag == 2
            IsSearchingFirst = 1;
%             vision_dT_T_last = otherMakers_k.ContinuesLastTime(i); % 对应连续点的时间
            vision_dT_k_last = otherMakers_k.ContinuesLastK(i); % 对应连续点的序号
            inertial_dT_k_last = VisionK_to_InertialK(vision_dT_k_last);
            % 第一个马克点搜索的关键：以当前马克点对应的连续点假设为跟踪成功点
            trackedMakerPosition_last_k_dT = otherMakers_k.ContinuesLastPosition(:,i) ; 
            
            [ trackedMakerPosition_k_OK,TrackFlag,min_dT_k,dPError_dT_xy,dPError_dT_z,dP_Inertial_xyNorm,dP_Inertial_z ] = Track_dT_Judge...
                ( otherMakers_k,InertialPosition,inertial_k,inertial_dT_k_last,trackedMakerPosition_last_k_dT );
            JudgeIndex.dPError_dT_xy = dPError_dT_xy ;
            JudgeIndex.dPError_dT_z = dPError_dT_z ;
            JudgeIndex.dP_Inertial_xyNorm = dP_Inertial_xyNorm ;
            JudgeIndex.dP_Inertial_z = dP_Inertial_z;
            if ~isnan(trackedMakerPosition_k_OK)
               fprintf('第一个马克点搜索成功  k_vision = %d dPError_dT_xy = %0.3f,  dPError_dT_z = %0.3f \n ',k_vision,dPError_dT_xy,dPError_dT_z); 
               return;
            end        
        end        
    end
    if IsSearchingFirst==0
        % 既找不到之前跟踪成功的点，又找不到连续的点，继续找
        fprintf('搜索第一个点：等待足够多连续性的点 k_vision = %d \n ',k_vision)
        return; 
    end
else
    % 之前有跟踪成功的数据
    trackedMakerPosition_last_k_dT = trackedMakerPosition(:,vision_dT_k_last) ;
    [ trackedMakerPosition_k_OK,TrackFlag,min_dT_k,dPError_dT_xy,dPError_dT_z,dP_Inertial_xyNorm,dP_Inertial_z ] = Track_dT_Judge( otherMakers_k,InertialPosition,inertial_k,inertial_dT_k_last,trackedMakerPosition_last_k_dT );
    JudgeIndex.dPError_dT_xy = dPError_dT_xy ;
    JudgeIndex.dPError_dT_z = dPError_dT_z ;
    JudgeIndex.dP_Inertial_xyNorm = dP_Inertial_xyNorm ;
    JudgeIndex.dP_Inertial_z = dP_Inertial_z;
end
if ~isnan(trackedMakerPosition_k_OK)    
   return;  % 跟踪OK 
end
otherMakersPosition_k_min = otherMakersPosition_k(:,min_dT_k);


      
%% dS 位移长度段的位移差：同时考虑位移差大小和方向
[ trackedMakerPosition_k_OK,TrackFlag,dPError_dS_xyNorm,angleErr_dS ] = Track_dS_Judge...
    ( InertialPosition,inertial_k,moveDistance,trackedMakerPosition,otherMakersPosition_k_min,TrackFlag ) ;
JudgeIndex.dPError_dS_xyNorm = dPError_dS_xyNorm  ;
JudgeIndex.angleErr_dS = angleErr_dS  ;

%% 马克点判定 4) 寻找惯性运动 dS 长度(大于 moveDistance)，且马克点跟踪成功的是时刻，判定 dP_Inertial 和 dP_Vision：
function [ trackedMakerPosition_k_OK,TrackFlag,dPError_dS_xyNorm,angleErr_dS ] = Track_dS_Judge...
    ( InertialPosition,inertial_k,moveDistance,trackedMakerPosition,otherMakersPosition_k_min,TrackFlag )
global  makerTrackThreshold
trackedMakerPosition_k_OK=  NaN ;
%% dS 位移长度段的位移差：同时考虑位移差大小和方向
% find the point which moved moveDistance
dS_Inertial_last_k = SearchDistanceK( InertialPosition,inertial_k,moveDistance,trackedMakerPosition ) ;  
if isnan(dS_Inertial_last_k)
    % 找不到运动大于 dS 长度且trackedMakerPosition中跟踪到的点
%     fprintf('找不到运动固定长度的点，不进行验证<2>，跟踪失败。\n');
    dPError_dS_xyNorm = -0.2 ;    % 给负值表示没有找到
    angleErr_dS = -10*pi/180;
    TrackFlag = -1.29 ;
    return; 
end
dS_Vision_last_k = InertialK_to_VisionK(dS_Inertial_last_k) ;    

dP_Inertial = InertialPosition(:,inertial_k) - InertialPosition(:,dS_Inertial_last_k);
dP_Vision = otherMakersPosition_k_min - trackedMakerPosition(:,dS_Vision_last_k);
dPError_dS = dP_Inertial-dP_Vision ;
dPError_dS(3) = 0 ; %  暂时没有记录脚，不用高度信息
dPError_dS_xyNorm = normest(dPError_dS(1:2)) ;
temp = dP_Inertial'*dP_Vision / normest(dP_Inertial) / normest(dP_Vision) ;
angleErr_dS = acos(temp) ;

if dPError_dS_xyNorm < makerTrackThreshold.MaxPositionError_dS  && angleErr_dS < makerTrackThreshold.MaxDisplaceAngle
    % 距离和角度 满足
    trackedMakerPosition_k_OK = otherMakersPosition_k_min ;
    TrackFlag = 4+TrackFlag ;
%     fprintf('3.1）3.2） 模=%0.3f，角度差=%0.3f，跟踪OK \n',normest(dPError_dS),angleErr_dS*180/pi);
else
    TrackFlag = -TrackFlag ;
%     fprintf('3.1）3.2） 模=%0.3f，角度差=%0.3f，跟踪失败 \n',normest(dPError_dS),angleErr_dS*180/pi);
end

%% 马克点判定 3) 短时间（dT=3）运动的过程，惯性和视觉位移差 dPError_dT 进行判定， dPError_dT(i) = normest(dP_Inertial-dP_Vision)

function [ trackedMakerPosition_k_OK,TrackFlag,min_dT_k,dPError_dT_xy,dPError_dT_z,dP_Inertial_xyNorm,dP_Inertial_z ] = Track_dT_Judge( otherMakers_k,InertialPosition,inertial_k,inertial_dT_k_last,trackedMakerPosition_last_k_dT )
global  makerTrackThreshold
trackedMakerPosition_k_OK = NaN ;
otherMakersPosition_k = otherMakers_k.Position ;
TrackFlag = 0;

M = size(otherMakersPosition_k,2) ;
dP_Inertial = InertialPosition(:,inertial_k) - InertialPosition(:,inertial_dT_k_last);
dP_Inertial_xy = dP_Inertial(1:2);
dP_Inertial_z = dP_Inertial(3);
dPError_dT  = zeros(1,M);
dPError_dT_xy  = zeros(1,M);
dPError_dT_z  = zeros(1,M);
dP_Vision = zeros(3,M);
for i=1:M
    otherMakersPosition_k_i = otherMakersPosition_k(:,i);
    dP_Vision(:,i) = otherMakersPosition_k_i - trackedMakerPosition_last_k_dT;
    dPError_dT(i) = normest(dP_Inertial-dP_Vision(:,i)) ;
    dPError_dT_xy(i) = normest(dP_Inertial_xy-dP_Vision(1:2,i)) ;
    dPError_dT_z(i) = normest(dP_Inertial_z-dP_Vision(3,i)) ;
end
[dPError_dT_xy,min_dT_k] = min(dPError_dT_xy,[],2);  % 不考虑高度
dPError_dT_z = dPError_dT_z(min_dT_k);
% 惯性位移大小 ： 评价静止还是运动
otherMakersPosition_k_min = otherMakersPosition_k(:,min_dT_k);
dP_Inertial_Norm = normest(dP_Inertial) ;  % 暂时没有记录脚，无法用高度信息
dP_Inertial_xyNorm = normest(dP_Inertial(1:2)) ;
dP_Inertial_z = dP_Inertial(3) ;
if dP_Inertial_xyNorm < makerTrackThreshold.MaxStaticDisp_dT  
    %% 准静止状态，用dS位移判断
    % 马克点明显运动，跟踪失败
    if dPError_dT > makerTrackThreshold.PositionErrorBear_dT
        TrackFlag = 1.31;
        return;
    end
    if otherMakers_k.ContinuesFlag(1,min_dT_k)==1
        trackedMakerPosition_k_OK = otherMakersPosition_k(:,min_dT_k) ;
        TrackFlag = 3.1;
        return;
    end
else
    %% 运动状态：
    % 静止点直接剔除
    if normest( dP_Vision(:,min_dT_k) )<makerTrackThreshold.MaxStaticDisp_dT
        TrackFlag=1.35;
        return;
    end
    if dPError_dT < makerTrackThreshold.PositionErrorBear_dT
        trackedMakerPosition_k_OK = otherMakersPosition_k_min ;
        TrackFlag = 3.5;
    %     fprintf( '3.5）惯性视觉位移差长度=%0.4f ，跟踪OK\n',normest(dPError_dT) );
        return;
    end
end


%% 马克点连续性判断
function [ otherMakers_k,dPi_ConJudge ] = ContinuesJudge( otherMakers_k,otherMakers_k_last,trackedMakerPosition,k_vision )


% 前一时刻跟踪成功时，给出当前点是否相对跟踪成功的点连续。  Continues = 1
% 前一时刻跟踪失败时，给出当前每个点是否为连续点的结果,且记录着个点能连续到往前最早（但不超过dT）的点的位置和时间。
%        Continues = 2 ，

global inertialFre visionFre makerTrackThreshold moveDistance

M = otherMakers_k.otherMakersN ;
otherMakers_k.ContinuesFlag = zeros(1,M) ; % 不连续
otherMakers_k.ContinuesLastPosition = NaN(3,M) ;
otherMakers_k.ContinuesLastTime = NaN(1,M) ;
otherMakersPosition_k = otherMakers_k.Position ;    
if k_vision>1 
    
    if ~isnan(trackedMakerPosition(1,k_vision-1))
        %% 只判断当前马克点是否与千食客跟踪成功的马克点连续
        trackedMakerPosition_kLast = trackedMakerPosition(:,k_vision-1) ;
        otherMakersPosition_k_Dis = otherMakersPosition_k-repmat(trackedMakerPosition_kLast,1,M) ;
        otherMakersPosition_k_Dis_Norm = zeros(1,M);
        for j=1:M
            otherMakersPosition_k_Dis_Norm(j) = normest(otherMakersPosition_k_Dis(:,j));
        end
        [dPi_ConJudge,minCon_k] = min(otherMakersPosition_k_Dis_Norm);  
        if dPi_ConJudge < makerTrackThreshold.MaxContinuesDisplacement
    %         trackedMakerPosition_k_OK = otherMakersPosition_k(:,m) ;
    %         TrackFlag = 1;
    %         fprintf('马克点连续：位移=%0.4f，跟踪OK \n',Min_otherMakersPosition_k_Dis_Norm);
            
            otherMakers_k.ContinuesFlag(1,minCon_k) = 1 ; % 连续，且是与跟踪成功马克点连续
            otherMakers_k.ContinuesLastPosition(:,minCon_k) = trackedMakerPosition_kLast ;
            otherMakers_k.ContinuesLastTime(minCon_k) = otherMakers_k_last.time ;
            otherMakers_k.ContinuesLastK(minCon_k) = k_vision-1 ;
        end
        
    else
        %% 判断当前马克点是否为连续马克点，记录每个点对应的最早（但不超过dT）连续点
        M_last = otherMakers_k_last.otherMakersN ;
        % 一共有 M*M_last 种组合
        for i=1:M
            dPi = repmat(otherMakers_k.Position( :,i ),1,M_last)- otherMakers_k_last.Position ;
            dPiNorm = normest(dPi);
            [dPi_ConJudge,min_i] = min(dPiNorm);   
            if normest(dPi_ConJudge) < makerTrackThreshold.MaxContinuesDisplacement
                %  otherMakers_k.Position( :,i ) 与 otherMakers_k_last.Position(:,min_i) 连续
                % 找到一个连续的点，记录上一点
                otherMakers_k.ContinuesFlag(i) = 2 ; % 连续，和跟踪失败的点连续
                % 如果前一个点为连续点，则将前一个点的连续记录传递过来
                if otherMakers_k_last.ContinuesFlag(min_i) == 2
                    otherMakers_k.ContinuesLastK(i) = otherMakers_k_last.ContinuesLastK(min_i) ; % 传递记录上一个时刻存储的连续信息
                    otherMakers_k.ContinuesLastPosition(:,i) = otherMakers_k_last.ContinuesLastPosition(:,min_i) ;
                    otherMakers_k.ContinuesLastTime(i) = otherMakers_k_last.ContinuesLastTime(min_i);
                elseif otherMakers_k_last.ContinuesFlag(min_i) == 0
                    otherMakers_k.ContinuesLastK(i) = k_vision-1 ; % 直接记录上一个时刻
                    otherMakers_k.ContinuesLastPosition(:,i) = otherMakers_k_last.Position( :,min_i ) ;
                    otherMakers_k.ContinuesLastTime(i) = otherMakers_k_last.time( min_i ) ;                    
                elseif otherMakers_k_last.ContinuesFlag(min_i) == 1
                    % 上一时刻跟踪失败，但是上上时刻跟踪成功，且上一时刻与上上时刻连续（其实应该也属于跟踪成功，由于其他指标不自信而没有认定）
                    % 同时当前点又与上一时刻连续，则可认为当前时刻与上上时刻连续
                    otherMakers_k.ContinuesFlag(i) = 1 ;
                    otherMakers_k.ContinuesLastK(i) = otherMakers_k_last.ContinuesLastK(min_i) ; % 传递记录上一个时刻存储的连续信息
                    otherMakers_k.ContinuesLastPosition(:,i) = otherMakers_k_last.ContinuesLastPosition(:,min_i) ;
                    otherMakers_k.ContinuesLastTime(i) = otherMakers_k_last.ContinuesLastTime(min_i);
                end
            else
                otherMakers_k.ContinuesFlag(i) = 0 ; % 不连续
                otherMakers_k.ContinuesLastPosition(:,i) = NaN ;
                otherMakers_k.ContinuesLastTime(i) = NaN ;
                otherMakers_k.ContinuesLastK(i) = NaN ;
            end
                        
        end
        
    end
else
    dPi_ConJudge=nan;
end



%% 查找惯性k时刻前运动了大于 dS 距离的最近的点
% 且该点trackedMakerPosition跟踪成功
function dS_Inertial_last_k = SearchDistanceK( InertialPosition,kCurrent,dS,trackedMakerPosition )
global inertialFre visionFre
stepT = 1 ;  % 搜索步长时间
MaxSearchT = 60 ; % 最长搜索时间长度
stepK = fix(inertialFre*stepT) ; % 搜索步长

maxSearchK = min( kCurrent-1,fix(inertialFre*MaxSearchT) );
InertialPosition_kSea = InertialPosition(:,kCurrent);
dS_Inertial_last_k = NaN ;     % 默认设置：寻找失败
%wh = waitbar(0,'SearchDistanceK');
for i=1:stepK:maxSearchK
    dP = InertialPosition(:,kCurrent-i) - InertialPosition_kSea ;
    distance = normest( dP ); 
    if distance > dS
        kCurrent_Vision = InertialK_to_VisionK(kCurrent-i);
        if ~isnan(trackedMakerPosition(1,kCurrent_Vision))            
            % 距离满足，且 trackedMakerPosition 跟踪成功
            dS_Inertial_last_k = kCurrent-i ;
            break;
        end
    end
 %   waitbar(i/kCurrent);
end
%close(wh)
disp('')

