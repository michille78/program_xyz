%% xyz 2015.5.25
%% otherMakers 中马克点识别：在多个马克点中找到正确的马克点
% trackedMakerPosition ： [3*N] 每个时刻跟踪成功马克点位置，跟踪失败 NaN(3,1)
% trackedMarkerVelocity ： [5*N] 跟踪成功马克点的速度，前三列xyz速度，
    % 	trackedMarkerVelocity(4,:)为xy平面内马克点速度模，trackedMarkerVelocity(5,:)为xy平面内马克点速度与[0 1 0] 的夹角

%% 跟踪测量见说明文档

%% 判断思路：比较2个相对位移矢量：1）dT(3 sec)运动时间时  2）dS（1m）运动位移长度时
% 1)dT(3 sec)时间内，惯性和视觉位移向量的大小差<0.1m，方向差<60°（当位移矢量长度小于0.2m时不比较方向）

function [ trackedMakerPosition,trackedMakerPosition_InertialTime,trackedMarkerVelocity,INSVNSCalib_VS_k,InertialPosition ] = GetRightOtherMaker( otherMakers,InertialData )
global  makerTrackThreshold moveDistance INSVNSCalibSet
global otherMakersTime  inertialTime 
global visionFre  inertialFre
global   INSMarkH0  VNSMarkH0
%% load data
visionFre = otherMakers(1).frequency ;
MarkerSet= otherMakers(1).MarkerSet ;
inertialTime = InertialData.time ;
inertialFre = InertialData.frequency ;

%% 阈值参数设置
moveTime = 2 ;          % sec 轨迹连续判断时间步长
moveDistance = 0.5 ;      % m   轨迹连续判断位移步长  （经验值建议0.4m-0.7m）
MaxMoveSpeed = 1.5 ; % m/s  马克点运动允许的最大速度，超过这个速度则认为不连续
makerTrackThreshold.moveTime = moveTime ;
makerTrackThreshold.MaxMoveTime = 3 ;
makerTrackThreshold.moveDistance = moveDistance ;
makerTrackThreshold.MaxContinuesDisplacement = min( 1/visionFre*MaxMoveSpeed,0.1) ; % 马克点连续判断最大位移模
makerTrackThreshold.PositionErrorBear_dT = 0.05*moveTime;   % 固定时间的最大运动距离误差（第一步）：位移差在这个范围内的，直接判定<校验1>通过
makerTrackThreshold.ContinuesTrackedMagnifyRate = 1.3 ;      % 当连续于跟踪成功的点时，放大PositionErrorBear_dT
MaxStaticSpeed = 0.1 ; % m/s 静止时，允许惯性最大测量误差
makerTrackThreshold.MaxStaticDisp_dT = max(MaxStaticSpeed*moveTime,0.02) ;           % 固定时间的最大运动距离误差（第二步）：（第一步不通过后）位移差的长度是惯性位移长度的MaxPositionError_dT倍以内
makerTrackThreshold.MaxPositionError_dS = moveDistance*0.7;     % 运动固定距离位移的最大运动距离误差：运动距离的50% （主要是依赖角度约束）
makerTrackThreshold.Max_dPAngle_dS = 20*pi/180 ;      % 运动固定距离位移的最大位移方向角度差

makerTrackThreshold.MaxMarkHighChange = 0.4 ;      % m 惯性与视觉目标马克点高度差变化最大范围，用于剔除高度相差较大的点

makerTrackThreshold.MaxHighMoveErrRate = 0.3 ;  %  高度方向变化大时，若变化比例小于这个值，直接认定跟踪OK
makerTrackThreshold.BigHighMove = 0.2 ;         % m 大于这个值则认为高度方向变化大
%% 坐标系标定参数
INSVNSCalibSet.Min_xyNorm_Calib = 0.3 ; % m  用于标定的数据的最小运动位移长度
INSVNSCalibSet.MaxTime_Calib = 2  ;  % sec  用于标定的数据的最长时间
INSVNSCalibSet.MaxVXY_DirectionChange_Calib = 30*pi/180 ;     % ° XY平面速度方向变化最大范围
INSVNSCalibSet.MaxVZ_Calib = 0.1 ;     % m/s Z方向速度最大绝对值
INSVNSCalibSet.MinVXY_Calib = 0.2;   	% m/s XY 平面速度模最小绝对值
INSVNSCalibSet.angleUniformityErr = 10*pi/180 ; % ° 位移矢量方向均匀性误差
% 速度计算
INSVNSCalibSet.dT_CalV_Calib = 0.5 ; % 计算速度时间步长（标定位移数据选择）
INSVNSCalibSet.MinXYVNorm_CalAngle = 0.1 ;  %  m/s xy速度模大于这个值才计算速度的反向

switch MarkerSet 
    case 'Head'
        InertialPosition = InertialData.HeadPosition ;
        
    case 'Hip'
        InertialPosition = InertialData.HipPosition ;
        
end
HipQuaternion = InertialData.HipQuaternion ;
HeadQuaternion = InertialData.HeadQuaternion ;

INSMarkH0 = NaN ;
VNSMarkH0 = NaN ;

dT_Ninertial = fix(moveTime*inertialFre) ;

%%
MarkerTN = length(otherMakers);
InertialN = size(InertialPosition,2);
trackedMakerPosition = NaN(3,MarkerTN); % 判断成功的马克点位置
trackedMakerPosition_InertialTime = NaN( 3,InertialN );
%% 先要求第一个点已知 
% trackedMakerPosition(:,1)  = otherMakers(1).Position(:,1) ;

% otherMakersNew = struct;
TrackFlag = zeros(1,MarkerTN);

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
angleErr_dT_Min = NaN(1,MarkerTN); 
ConTrackedFailed = zeros(1,MarkerTN);
INSVNSMarkHC_Min = NaN(1,MarkerTN); 
trackedMarkerVelocity = NaN(5,MarkerTN);  
INSVNSCalib_VS_k = [];

IsCalibDataEnough = 0;

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
%         if k>2 && isnan(trackedMakerPosition(1,k-2))
%             if sum(otherMakers_k_last.ContinuesFlag==1)~=0
%                disp('err') 
%             end
%         end
    else
        otherMakers_k_last = [];
    end
    
     
    [ trackedMakerPosition(:,k),otherMakersNew_k,TrackFlag(k),JudgeIndex  ] = JudgeMaker...
        ( otherMakers_k,otherMakers_k_last,k,inertial_k,trackedMakerPosition,InertialPosition,inertial_dT_k_last,vision_dT_k_last ) ;
    otherMakers(k)=otherMakersNew_k;
    dPi_ConJudge(k) = JudgeIndex.dPi_ConJudge  ;
    dPError_dT_xy(k) = JudgeIndex.dPError_dT_xy ;
    dPError_dT_z(k) = JudgeIndex.dPError_dT_z ;
    dPError_dS_xyNorm(k) = JudgeIndex.dPError_dS_xyNorm ;
    dP_Inertial_xyNorm(k) = JudgeIndex.dP_Inertial_xyNorm ;
    angleErr_dT_Min(k) = JudgeIndex.angleErr_dT_Min ;
    angleErr_dS(k) = JudgeIndex.angleErr_dS ;
    INSVNSMarkHC_Min(k) = JudgeIndex.INSVNSMarkHC_Min ;
       
    if sum(otherMakersNew_k.ContinuesFlag==1)~=0 && isnan(trackedMakerPosition(1,k))
        ConTrackedFailed(k) = 1 ;
    else
        ConTrackedFailed(k) = 0 ;
    end
    
    %% 求  INSMarkH0  VNSMarkH0
    if isnan(INSMarkH0) && ~isnan(trackedMakerPosition(1,k))
        fprintf( '第一个点搜素成功 time = %0.1f sec',k/visionFre );
        % 检测Hip的俯仰和横滚小才取值！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！
        INSMarkH0 = - InertialPosition(3,inertial_k);
        VNSMarkH0 = - trackedMakerPosition(3,k) ;
%         HipQuaternion_k = HipQuaternion( :,inertial_k );
%         CHip_k = Q2C(HipQuaternion_k);
%         
%         C_HipLUF_NED0 = RotateX(pi/2) * RotateY(-pi/2);  % Hip 的左上前系 到 NED的0姿态系
%         C_NED_HipNED0 = C_HipLUF_NED0 * CHip_k ;               
%         Attitude = C2Euler( C_NED_HipNED0,'ZYX' )*180/pi
        %% 第一次跟踪马克点成功
        % 标定视觉与光学的原点（不考虑方向差）
        Xrw_r = trackedMakerPosition(1:2,k) - InertialPosition(1:2,inertial_k);
        Xrw_r = [Xrw_r;0];
        N_otherMakers = length( otherMakers );
        for i=1:N_otherMakers
            if ~isempty(otherMakers(i).Position)
                m = size( otherMakers(i).Position,2 );
                otherMakers(i).Position = otherMakers(i).Position - repmat(Xrw_r,1,m) ;
                trackedMakerPosition(:,i) = trackedMakerPosition(:,i) - Xrw_r ;
            end
        end
    end
    %% 求速度
    [ trackedMarkerVelocity_k,k_calV ] = VisionMarkVelocity( trackedMakerPosition,k ) ;
    if k_calV>0 
        trackedMarkerVelocity(:,k_calV) = trackedMarkerVelocity_k ;
        if IsCalibDataEnough==0
        %% 搜索可用于标定的数据
    %         dbstop in SearchCalibData
            [ INSVNSCalib_VS_k,IsCalibDataEnough,dX_Vision ] = SearchCalibData...
                ( INSVNSCalib_VS_k,trackedMarkerVelocity,trackedMakerPosition,k_calV ) ;
            if IsCalibDataEnough==1
                Crw = INSVNSCalib( INSVNSCalib_VS_k,dX_Vision,InertialPosition );
                 otherMakers = CompensateSecond( otherMakers,Crw  );
            end
        end        
    end
    %% 转成惯性马克点的时序
    
    trackedMakerPosition_InertialTime(:,inertial_k) = trackedMakerPosition(:,k) ;
    
    if mod(k,fix(MarkerTN/10))==0
        waitbar(k/MarkerTN);
    end
end
 close(wh);

%  return
 
figure('name','trackFlag')
plot(TrackFlag,'.')
% plot(otherMakersTime,TrackFlag,'.')
% xlabel('time sec')

figure('name','dPi_ConJudge')
plot(otherMakersTime,dPi_ConJudge)
temp = makerTrackThreshold.MaxContinuesDisplacement ;
line( [otherMakersTime(1) otherMakersTime(MarkerTN)],[temp temp],'color','r' )

xlabel('time sec')

figure('name','INSVNSMarkHC_Min')
plot(otherMakersTime,INSVNSMarkHC_Min)
temp = makerTrackThreshold.MaxMarkHighChange ;
line( [otherMakersTime(1) otherMakersTime(MarkerTN)],[temp temp],'color','r' )
temp = -makerTrackThreshold.MaxMarkHighChange ;
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

figure('name','angleErr_dT_Min')
plot(otherMakersTime,angleErr_dT_Min*180/pi)
temp = makerTrackThreshold.Max_dPAngle_dS*180/pi ;
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
temp = makerTrackThreshold.Max_dPAngle_dS*180/pi ;
line( [otherMakersTime(1) otherMakersTime(MarkerTN)],[temp temp],'color','r' )
xlabel('time sec')

figure('name','ConTrackedFailed')
plot(ConTrackedFailed)



FailTrackFlagNum = sum( TrackFlag<=0 );
fprintf( 'FailTrackFlagNum=%d ( %0.3f ) \n',FailTrackFlagNum,FailTrackFlagNum/length(TrackFlag) );


function otherMakers = CompensateSecond( otherMakers,Crw  )


N = length( otherMakers );

for k=1:N
    if ~isempty(otherMakers(k).Position)
        otherMakers(k).Position = Crw*otherMakers(k).Position ;
    end
end

%% Judge which is the right maker
% 1) 固定运动时间位移判断：只判断位移差长度
% 2）固定运动距离位移判断：同时判断位移差长度和方向

function [ trackedMakerPosition_k_OK,otherMakers_k,TrackFlag,JudgeIndex ] = JudgeMaker...
( otherMakers_k,otherMakers_k_last,k_vision,inertial_k,trackedMakerPosition,InertialPosition,inertial_dT_k_last,...
            vision_dT_k_last )
global  visionFre makerTrackThreshold moveDistance

JudgeIndex.dPi_ConJudge = NaN ;
JudgeIndex.dPError_dT_xy = NaN ;
JudgeIndex.dPError_dT_z = NaN ;
JudgeIndex.dPError_dS_xyNorm = NaN ;
JudgeIndex.dP_Inertial_xyNorm = NaN ;
JudgeIndex.angleErr_dS = NaN  ;
JudgeIndex.angleErr_dT_Min = NaN  ;
JudgeIndex.INSVNSMarkHC_Min = NaN  ;


trackedMakerPosition_k_OK = NaN;  % 跟踪马克点失败置 NaN
TrackFlag = 0;

M = otherMakers_k.otherMakersN ;
otherMakersPosition_k = otherMakers_k.Position ;


if isempty(otherMakersPosition_k)
    return;
end

%% 高度判断
[ otherMakers_k,trackedMakerPosition_k_OK,TrackFlag,INSVNSMarkHC_Min ] = Track_High_Judge...
    ( otherMakers_k,inertial_k,InertialPosition ) ;
JudgeIndex.INSVNSMarkHC_Min = INSVNSMarkHC_Min  ;
M = otherMakers_k.otherMakersN ;  % 点的个数可能被减少了
if TrackFlag == -1
    return;
end
%% 马克点连续性判断
[ otherMakers_k,dPi_ConJudge ] = ContinuesJudge( otherMakers_k,otherMakers_k_last,trackedMakerPosition,k_vision );
JudgeIndex.dPi_ConJudge = dPi_ConJudge ;

%% dT 时间段的位移差：只考虑位移矢量大小
% 如果 vision_dT_k_last 没跟踪成功，把时间往前推直到找到跟踪成功的点。但是不能往前推超过 Max_dT 时间。搜索了
% Max_dT时间还没有找到的话，
 
while vision_dT_k_last>1 && isnan( trackedMakerPosition(1,vision_dT_k_last) ) % trackedMakerPosition(1) 必须已知（不为nan）
    vision_dT_k_last = vision_dT_k_last-1 ;
    inertial_dT_k_last = VisionK_to_InertialK(vision_dT_k_last);
    if (k_vision-vision_dT_k_last)/visionFre > makerTrackThreshold.MaxMoveTime
%         fprintf( '放弃搜索，作为新点重新搜索  k_vision = %d , vision_dT_k_last = %d \n',k_vision,vision_dT_k_last );
        break;
    end
end
if isnan( trackedMakerPosition(1,vision_dT_k_last) )
    %% 寻找第一个点（第一个点：怎么都找不到之前跟踪成功的点）
    % 用其他连续点计算，分析当前马克点中每一个连续点，如果某个连续点的 dT 判断通过，则认为这是第一个点
    IsSearchingFirst = 0;
    for i=1:M
        if  otherMakers_k.ContinuesFlag(i) == 2 || otherMakers_k.ContinuesFlag(i) == 1
            IsSearchingFirst = 1;
%             vision_dT_T_last = otherMakers_k.ContinuesLastTime(i); % 对应连续点的时间
            vision_dT_k_last = otherMakers_k.ContinuesLastK(i); % 对应连续点的序号
            inertial_dT_k_last = VisionK_to_InertialK(vision_dT_k_last);
            % 第一个马克点搜索的关键：以当前马克点对应的连续点假设为跟踪成功点
            trackedMakerPosition_last_k_dT = otherMakers_k.ContinuesLastPosition(:,i) ; 
            
            [ trackedMakerPosition_k_OK,TrackFlag,min_dT_k,dPError_dT_xy,dPError_dT_z,dP_Inertial_xyNorm,angleErr_dT_Min ] = Track_dT_Judge...
                ( otherMakers_k,InertialPosition,inertial_k,inertial_dT_k_last,trackedMakerPosition_last_k_dT );
            JudgeIndex.dPError_dT_xy = dPError_dT_xy ;
            JudgeIndex.dPError_dT_z = dPError_dT_z ;
            JudgeIndex.dP_Inertial_xyNorm = dP_Inertial_xyNorm ;
            JudgeIndex.angleErr_dT_Min = angleErr_dT_Min;
            if ~isnan(trackedMakerPosition_k_OK)
%                 fprintf('第一个马克点搜索成功  k_vision = %d dPError_dT_xy = %0.3f,  dPError_dT_z = %0.3f \n ',k_vision,dPError_dT_xy,dPError_dT_z); 
               return;
            end        
        end        
    end
    if IsSearchingFirst==0
        % 既找不到之前跟踪成功的点，又找不到连续的点，继续找
%         fprintf('搜索第一个点：等待足够多连续性的点 k_vision = %d \n ',k_vision)
        return; 
    end
else
    % 之前有跟踪成功的数据
    trackedMakerPosition_last_k_dT = trackedMakerPosition(:,vision_dT_k_last) ;
    [ trackedMakerPosition_k_OK,TrackFlag,min_dT_k,dPError_dT_xy,dPError_dT_z,dP_Inertial_xyNorm,angleErr_dT_Min ] = ...
        Track_dT_Judge( otherMakers_k,InertialPosition,inertial_k,inertial_dT_k_last,trackedMakerPosition_last_k_dT );
    JudgeIndex.dPError_dT_xy = dPError_dT_xy ;
    JudgeIndex.dPError_dT_z = dPError_dT_z ;
    JudgeIndex.dP_Inertial_xyNorm = dP_Inertial_xyNorm ;
    JudgeIndex.angleErr_dT_Min = angleErr_dT_Min;
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

%% 高度判定
% （1.1）惯性目标关节与视觉目标马克点高度差（INSVNSMarkHC） =  INSVNSMarkL *
% cos(thita)。INSVNSMarkL 为常数，在初始时刻刻计算得到。
%  INSVNSMarkHC 计算方法：当前高度-直立时的高度
function [ otherMakers_k,trackedMakerPosition_k_OK,TrackFlag,INSVNSMarkHC_Min ] = Track_High_Judge...
    ( otherMakers_k,inertial_k,InertialPosition )
global  makerTrackThreshold   INSMarkH0  VNSMarkH0

trackedMakerPosition_k_OK = NaN;
INSVNSMarkHC_Min = NaN ;
TrackFlag = 0 ;    
if isnan(INSMarkH0)
       
    return ;
end

M = otherMakers_k.otherMakersN ;
otherMakersPosition_k = otherMakers_k.Position ;

INSVNSMarkHC = zeros(1,M);
for i=1:M
    INSVNSMarkHC(i) = ( -InertialPosition(3,inertial_k) + otherMakersPosition_k(3,i) )-( INSMarkH0-VNSMarkH0 ) ;   
end
% 高度差最小的点
[ INSVNSMarkHC_Min,min_i ] = min( abs(INSVNSMarkHC) );
% 将高度差误差大的点剔除
invalid_i = 0;
for i=1:M
    if abs(INSVNSMarkHC(i)) > makerTrackThreshold.MaxMarkHighChange
        % 将高度差误差大的点剔除        
        otherMakers_k.Position(:,i-invalid_i) = [];
        otherMakers_k.otherMakersN = otherMakers_k.otherMakersN-1 ;
        invalid_i = invalid_i+1 ;
    end
end

%% 通过高度剔除给踢完了
if otherMakers_k.otherMakersN == 0
   TrackFlag = -1;
   trackedMakerPosition_k_OK = NaN;
   return;
end



%% 马克点判定 4) 寻找惯性运动 dS 长度(大于 moveDistance)，且马克点跟踪成功的是时刻，判定 dP_Inertial 和 dP_Vision：
function [ trackedMakerPosition_k_OK,TrackFlag,dPError_dS_Norm,angleErr_dS ] = Track_dS_Judge...
    ( InertialPosition,inertial_k,moveDistance,trackedMakerPosition,otherMakersPosition_k_min,TrackFlag )
global  makerTrackThreshold
trackedMakerPosition_k_OK=  NaN ;
%% dS 位移长度段的位移差：同时考虑位移差大小和方向
% find the point which moved moveDistance
dS_Inertial_last_k = SearchDistanceK( InertialPosition,inertial_k,moveDistance,trackedMakerPosition ) ;  
if isnan(dS_Inertial_last_k)
    % 找不到运动大于 dS 长度且trackedMakerPosition中跟踪到的点
%     fprintf('找不到运动固定长度的点，不进行验证<2>，跟踪失败。\n');
    dPError_dS_Norm = -0.2 ;    % 给负值表示没有找到
    angleErr_dS = -10*pi/180;
    TrackFlag = -1.4 ;
    return; 
end
dS_Vision_last_k = InertialK_to_VisionK(dS_Inertial_last_k) ;    

dP_Inertial = InertialPosition(:,inertial_k) - InertialPosition(:,dS_Inertial_last_k);
dP_Vision = otherMakersPosition_k_min - trackedMakerPosition(:,dS_Vision_last_k);
dPError_dS = dP_Inertial-dP_Vision ;
dPError_dS_Norm = normest(dPError_dS) ;
temp = dP_Inertial'*dP_Vision / normest(dP_Inertial) / normest(dP_Vision) ;
angleErr_dS = acos(temp) ;

if dPError_dS_Norm < makerTrackThreshold.MaxPositionError_dS  && angleErr_dS < makerTrackThreshold.Max_dPAngle_dS
    % 距离和角度 满足
    trackedMakerPosition_k_OK = otherMakersPosition_k_min ;
    TrackFlag = 4+TrackFlag ;
%     fprintf('3.1）3.2） 模=%0.3f，角度差=%0.3f，跟踪OK \n',normest(dPError_dS),angleErr_dS*180/pi);
else
    TrackFlag = -TrackFlag ;
%     fprintf('3.1）3.2） 模=%0.3f，角度差=%0.3f，跟踪失败 \n',normest(dPError_dS),angleErr_dS*180/pi);
end

%% 马克点判定 3) 短时间（dT=3）运动的过程，惯性和视觉位移差 dPError_dT 进行判定， dPError_dT(i) = normest(dP_Inertial-dP_Vision)

function [ trackedMakerPosition_k_OK,TrackFlag,min_dT_k,dPErrorNorm_dT_Min,dPError_dT_z_Min,dP_Inertial_xyNorm_Min,angleErr_dT_Min ]...
    = Track_dT_Judge( otherMakers_k,InertialPosition,inertial_k,inertial_dT_k_last,trackedMakerPosition_last_k_dT )
global  makerTrackThreshold
trackedMakerPosition_k_OK = NaN ;
otherMakersPosition_k = otherMakers_k.Position ;
TrackFlag = 0;

M = size(otherMakersPosition_k,2) ;
dP_Inertial = InertialPosition(:,inertial_k) - InertialPosition(:,inertial_dT_k_last);
dP_Inertial_xy = dP_Inertial(1:2);
dP_Inertial_z = dP_Inertial(3);
dPErrorNorm_dT  = zeros(1,M);
dPError_dT_xy  = zeros(1,M);
dPError_dT_z  = zeros(1,M);
dP_Vision = zeros(3,M);
angleErr_dT  = NaN(1,M);
angleErr_dT_Min = NaN;
%% 位移差 模， 角度差
for i=1:M
    otherMakersPosition_k_i = otherMakersPosition_k(:,i);
    dP_Vision(:,i) = otherMakersPosition_k_i - trackedMakerPosition_last_k_dT;
    % 位移差
    dPErrorNorm_dT(i) = normest(dP_Inertial-dP_Vision(:,i)) ;
    dPError_dT_xy(i) = normest(dP_Inertial_xy-dP_Vision(1:2,i)) ;
    dPError_dT_z(i) = normest(dP_Inertial_z-dP_Vision(3,i)) ;
    % 角度差
    temp = dP_Inertial'*dP_Vision(:,i) / normest(dP_Inertial) / normest(dP_Vision(:,i)) ;
    angleErr_dT(i) = acos(temp) ;
end



[dPErrorNorm_dT_Min,min_dT_k] = min(dPErrorNorm_dT,[],2);  % 取位移差最小的点判断
dPError_dT_z_Min = dPError_dT_z(min_dT_k);
% 惯性位移大小 ： 评价静止还是运动
otherMakersPosition_k_min = otherMakersPosition_k(:,min_dT_k);
dP_Inertial_Norm = normest(dP_Inertial) ;  
dP_Inertial_xyNorm_Min = normest(dP_Inertial(1:2)) ;
dP_Inertial_z_Min = dP_Inertial(3) ;
if dP_Inertial_Norm < makerTrackThreshold.MaxStaticDisp_dT  
    %% 准静止状态，用dS位移判断
    if dPErrorNorm_dT_Min > makerTrackThreshold.PositionErrorBear_dT
        % 马克点明显运动，剔除
        TrackFlag = -3.1 ;
    else
        % 惯性视觉运动位移差小，且连续于跟踪OK的马克点，则跟踪OK。
        %% 连续判断出错时会误判
        if otherMakers_k.ContinuesFlag(min_dT_k)==1
            trackedMakerPosition_k_OK = otherMakersPosition_k(:,min_dT_k) ;
            TrackFlag = 3.1;
            return;
        else
            % 惯性视觉运动位移差小，但不连续，进入4）判断
            TrackFlag = 1.31;
            return;
        end
    end
    
else
    %% 运动状态：
    if otherMakers_k.ContinuesFlag(min_dT_k)==1
       %% 与前一时刻跟踪成功的点连续则放宽要求
       PositionErrorBear_dT = makerTrackThreshold.PositionErrorBear_dT*makerTrackThreshold.ContinuesTrackedMagnifyRate ;
       Max_dPAngle_dS=  makerTrackThreshold.Max_dPAngle_dS*makerTrackThreshold.ContinuesTrackedMagnifyRate ;
       MaxHighMoveErrRate = makerTrackThreshold.MaxHighMoveErrRate*makerTrackThreshold.ContinuesTrackedMagnifyRate ;
    else
        PositionErrorBear_dT = makerTrackThreshold.PositionErrorBear_dT ;
       Max_dPAngle_dS=  makerTrackThreshold.Max_dPAngle_dS ;
       MaxHighMoveErrRate = makerTrackThreshold.MaxHighMoveErrRate ;
    end
    % 静止点直接剔除
    if normest( dP_Vision(:,min_dT_k) )<makerTrackThreshold.MaxStaticDisp_dT
        TrackFlag=1.35;
        return;
    end
    %% 高度变化大，水平方向不可信，直接通过高度判定
    if normest(dP_Inertial_z_Min) > makerTrackThreshold.BigHighMove
       HighMoveErr = dPError_dT_z(min_dT_k) / dP_Inertial_z_Min ;
       if HighMoveErr>0 && HighMoveErr < MaxHighMoveErrRate
           %% 高度方向位移误差比例小，跟踪OK
           TrackFlag = 3.9 ;
           trackedMakerPosition_k_OK = otherMakersPosition_k(:,min_dT_k) ;
           return;
       end
    end
    %% （角度差判断为主）当运动距离超过 moveDistance 时，判断角度差，并放宽位移差条件
    if dP_Inertial_Norm > makerTrackThreshold.moveDistance
       if angleErr_dT(min_dT_k) <  makerTrackThreshold.Max_dPAngle_dS && dPErrorNorm_dT_Min < makerTrackThreshold.MaxPositionError_dS
            trackedMakerPosition_k_OK = otherMakersPosition_k_min ;
            TrackFlag = 3.7;
            angleErr_dT_Min = angleErr_dT(min_dT_k) ;
            return;
       end
    end
    %% 位移差判断为主
    if dPErrorNorm_dT_Min < PositionErrorBear_dT  && angleErr_dT(min_dT_k) <  Max_dPAngle_dS*2
        trackedMakerPosition_k_OK = otherMakersPosition_k_min ;
        TrackFlag = 3.5;
    %     fprintf( '3.5）惯性视觉位移差长度=%0.4f ，跟踪OK\n',normest(dPError_dT) );
        return;
    end
    TrackFlag=1.37;
end

if TrackFlag==0
   disp('error')  
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
            
            otherMakers_k.ContinuesFlag(minCon_k) = 1 ; % 连续，且是与跟踪成功马克点连续
            otherMakers_k.ContinuesLastPosition(:,minCon_k) = trackedMakerPosition_kLast ;
            otherMakers_k.ContinuesLastTime(minCon_k) = otherMakers_k_last.time ;
            otherMakers_k.ContinuesLastK(minCon_k) = k_vision-1 ;
        end
        
    else
        %% 判断当前马克点是否为连续马克点，记录每个点对应的最早（但不超过dT）连续点
        M_last = otherMakers_k_last.otherMakersN ;
        if M_last==0
            % 上时刻无马克点
            for i=1:M
                otherMakers_k.ContinuesFlag(i) = 0 ; % 不连续
                otherMakers_k.ContinuesLastPosition(:,i) = NaN ;
                otherMakers_k.ContinuesLastTime(i) = NaN ;
                otherMakers_k.ContinuesLastK(i) = NaN ;
            end
            dPi_ConJudge=nan;
            return;
        end
        % 一共有 M*M_last 种组合
        for i=1:M
            dPi = repmat(otherMakers_k.Position( :,i ),1,M_last)- otherMakers_k_last.Position ;
            dPiNorm = zeros(1,M_last);
            for j=1:M_last
                dPiNorm(j) = normest(dPi(:,j));
            end
            
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
                    otherMakers_k.ContinuesLastTime(i) = otherMakers_k_last.time ;                    
                elseif otherMakers_k_last.ContinuesFlag(min_i) == 1 
                    % 与跟踪成功点连续检测成功，但跟踪识别失败的情况，传递到现在。如果传递时间超过2秒，则不再传递。
                    if (otherMakers_k_last.ContinuesLastTime(min_i)-otherMakers_k.time) > 20
                        otherMakers_k.ContinuesFlag(i) = 2 ;
                    else
                        otherMakers_k.ContinuesFlag(i) = 1 ;
                    end                    
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

