%% xyz 2015.5.25
%% otherMakers 中马克点识别：在多个马克点中找到正确的马克点
% trackedMakerPosition ： [3*N] 每个时刻跟踪成功马克点位置，跟踪失败 NaN(3,1)
% trackedMarkerVelocity ： [5*N] 跟踪成功马克点的速度，前三列xyz速度，
    % 	trackedMarkerVelocity(4,:)为xy平面内马克点速度模，trackedMarkerVelocity(5,:)为xy平面内马克点速度与[0 1 0] 的夹角

%% 跟踪测量见说明文档

%% 判断思路：比较2个相对位移矢量：1）dT(3 sec)运动时间时  2）dS（1m）运动位移长度时
% 1)dT(3 sec)时间内，惯性和视觉位移向量的大小差<0.1m，方向差<60°（当位移矢量长度小于0.2m时不比较方向）

function [ trackedMakerPosition_InertialTime_Out,otherMakers ] = ...
    GetRightOtherMaker( otherMakers,InertialPosition )
coder.extrinsic('fprintf');
coder.extrinsic('DrawTracedINSVNS');

global    visionFre inertialFre 
global      IsGetFirstMarker 
global      CalStartVN CalEndVN  CalStartIN  CalEndIN  

%% 阈值参数设置
persistent makerTrackThreshold INSVNSCalibSet 
if isempty(makerTrackThreshold)
    [ makerTrackThreshold,INSVNSCalibSet ] = SetConstParameters( );
end

IsGetFirstMarker = 0 ;

dT_Ninertial = fix(makerTrackThreshold.moveTime*inertialFre) ;

%% 在线/离线 第一次要求预定义变量大小
persistent   trackedMakerPosition_InertialTime  trackedMakerPosition TrackFlag trackedMarkerVelocity  InertialVelocity
persistent INSVNSCalib_VS_k  Calib_N_New  IsCalibDataEnough
MarkerTN = length(otherMakers);   %  视觉时间长度
InertialN = size(InertialPosition,2);

if isempty(InertialVelocity)
    TrackFlag = zeros(1,MarkerTN);
    trackedMakerPosition = NaN(3,MarkerTN); % 判断成功的马克点位置
    trackedMakerPosition_InertialTime = NaN( 3,InertialN );
    
    InertialVelocity = NaN(5,InertialN);
    trackedMarkerVelocity = NaN(5,MarkerTN); 
    
    INSVNSCalib_VS_k = NaN(2,50); % INSVNSCalib_VS_k（:,i）存储一段位移
    Calib_N_New = 0;   % 搜索乘的位移段个数
    IsCalibDataEnough = 0;
end
%% 这些判断指标只在离线时记录
persistent dPi_ConJudge dPError_dT_xy  dPError_dT_z  dPError_dS_xyNorm dP_Inertial_xyNorm  angleErr_dS  angleErr_dT_Min ConTrackedFailed INSVNSMarkHC_Min
if isempty(dPi_ConJudge)
    dPi_ConJudge = NaN(1,MarkerTN);   
    dPError_dT_xy = NaN(1,MarkerTN);  
    dPError_dT_z = NaN(1,MarkerTN);  
    dPError_dS_xyNorm = NaN(1,MarkerTN);  
    dP_Inertial_xyNorm = NaN(1,MarkerTN);  
    angleErr_dS = NaN(1,MarkerTN);  
    angleErr_dT_Min = NaN(1,MarkerTN); 
    ConTrackedFailed = zeros(1,MarkerTN);
    INSVNSMarkHC_Min = NaN(1,MarkerTN);               
end
persistent Crw
if isempty(Crw)
   Crw = eye(3); 
end
%% 求惯性速度
for k=CalStartIN:CalEndIN
    [ InertialVelocity_i,k_calV ] = CalVelocity( InertialPosition,k,inertialFre,INSVNSCalibSet.dT_CalV_Calib,INSVNSCalibSet.MinXYVNorm_CalAngle );
    if k_calV>0 
        InertialVelocity(:,k_calV) = InertialVelocity_i ;
    end
end
%% 单马克点跟踪 
coder.extrinsic('waitbar');
coder.extrinsic('close');
%  wh = waitbar(0,'SearchDistanceK');
for k=CalStartVN:CalEndVN
    otherMakers_k = otherMakers(k) ;  
    inertial_k = otherMakers_k.inertial_k ;
        
    %  last_dT_k
    inertial_dT_k_last = inertial_k - dT_Ninertial ;
    inertial_dT_k_last = max(inertial_dT_k_last,1);        

    if k>1
        otherMakers_k_last = otherMakers(k-1) ; 
        if coder.target('MATLAB') && isnan(otherMakers_k_last.ContinuesFlag) && otherMakers_k_last.otherMakersN>0
           fprintf('判定第%0.0f个点时，前一个otherMakers点没有结果\n',k); 
        end
        % otherMakers_k_last 必须是更新了判定结果的，检查一下
        
        
%         if k>2 && isnan(trackedMakerPosition(1,k-2))
%             if sum(otherMakers_k_last.ContinuesFlag==1)~=0
%                disp('err') 
%             end
%         end
    else
        otherMakers_k_last = GetEmpty_otherMakers(otherMakers_k) ;
    end

    [ trackedMakerPosition(:,k),otherMakersNew_k,TrackFlag(k),JudgeIndex  ] = JudgeMaker...
        ( otherMakers_k,otherMakers_k_last,k,inertial_k,trackedMakerPosition,InertialPosition,inertial_dT_k_last,...
        makerTrackThreshold ) ;
    if coder.target('MATLAB')
        dPi_ConJudge(k) = JudgeIndex.dPi_ConJudge  ;
        dPError_dT_xy(k) = JudgeIndex.dPError_dT_xy ;
        dPError_dT_z(k) = JudgeIndex.dPError_dT_z ;
        dPError_dS_xyNorm(k) = JudgeIndex.dPError_dS_xyNorm ;
        dP_Inertial_xyNorm(k) = JudgeIndex.dP_Inertial_xyNorm ;
        angleErr_dT_Min(k) = JudgeIndex.angleErr_dT_Min ;
        angleErr_dS(k) = JudgeIndex.angleErr_dS ;
        INSVNSMarkHC_Min(k) = JudgeIndex.INSVNSMarkHC_Min ;
    end
       
    if sum(otherMakersNew_k.ContinuesFlag==1)~=0 && isnan(trackedMakerPosition(1,k))
        ConTrackedFailed(k) = 1 ;
    else
        ConTrackedFailed(k) = 0 ;
    end
    
    %% 求  INSMarkH0  VNSMarkH0
    if isnan(makerTrackThreshold.INSMarkH0) && ~isnan(trackedMakerPosition(1,k))
        fprintf( '第一个点搜素成功 time = %0.1f sec, TrackFlag(%d) = %0.1f ,',k/visionFre,k,TrackFlag(k) );
        % 检测Hip的俯仰和横滚小才取值！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！
        makerTrackThreshold.INSMarkH0 = - InertialPosition(3,inertial_k);
        makerTrackThreshold.VNSMarkH0 = - trackedMakerPosition(3,k) ;
        
 
%         HipQuaternion_k = HipQuaternion( :,inertial_k );
%         HipAttitude = GetHipAttitude( HipQuaternion_k );
%         
%         HipQuaternion_k = Qinv( HipQuaternion_k ) ; % 四元数定义由 顺时针 改为 逆时针。
%         
%         CHip_k = Q2C(HipQuaternion_k);
%         
%         C_HipLUF_NED0 = RotateX(pi/2) * RotateY(-pi/2);  % Hip 的左上前系 到 NED的0姿态系
%         C_NED_HipNED0 = C_HipLUF_NED0 * CHip_k ;               
%         Attitude = C2Euler( C_NED_HipNED0,'ZYX' )*180/pi
        
        
        %% 第一次跟踪马克点成功
        % 标定视觉与光学的原点（不考虑方向差）
%         Xrw_r = trackedMakerPosition(1:2,k) - InertialPosition(1:2,inertial_k);
%         Xrw_r = [Xrw_r;0];
%         N_otherMakers = length( otherMakers );
%         for i=1:N_otherMakers
%             if ~isempty(otherMakers(i).Position)
%                 m = size( otherMakers(i).Position,2 );
%                 otherMakers(i).Position = otherMakers(i).Position - repmat(Xrw_r,1,m) ;
%                 trackedMakerPosition(:,i) = trackedMakerPosition(:,i) - Xrw_r ;
%             end
%         end
    end
    %% 求视觉速度
    [ trackedMarkerVelocity_k,k_calV ] = CalVelocity...
        ( trackedMakerPosition,k,visionFre,INSVNSCalibSet.dT_CalV_Calib,INSVNSCalibSet.MinXYVNorm_CalAngle );
    if k_calV>0 
        trackedMarkerVelocity(:,k_calV) = trackedMarkerVelocity_k ;
        if IsCalibDataEnough==0   % 只搜索一次
        %% 搜索可用于标定的数据
    %         dbstop in SearchCalibData
            Calib_N_Last = Calib_N_New;
            [ INSVNSCalib_VS_k,Calib_N_New,IsCalibDataEnough,dX_Vision ] = SearchCalibData...
                ( INSVNSCalib_VS_k,Calib_N_Last,trackedMarkerVelocity,trackedMakerPosition,k_calV,INSVNSCalibSet ) ;
            if IsCalibDataEnough==1
                fprintf( '第一个用于坐标系映射计算的点OK，time = %0.1f sec \n',k/visionFre );
                Crw = INSVNSCalib( INSVNSCalib_VS_k,Calib_N_New,dX_Vision,InertialPosition );                 
            end
        end        
    end
    otherMakersNew_k = CompensateSecond( otherMakersNew_k,Crw  );
    otherMakers(k)=otherMakersNew_k;     % 更新 otherMakers(k)
    %% 转成惯性马克点的时序
    
    trackedMakerPosition_InertialTime(:,inertial_k) = trackedMakerPosition(:,k) ;
%     
%     if mod(k,fix(MarkerTN/10))==0
%         waitbar(k/MarkerTN);
%     end
end
%  close(wh);
 %% Output
trackedMakerPosition_InertialTime_Out = trackedMakerPosition_InertialTime ;
 
 %% result analyse

if coder.target('MATLAB')  && (CalEndVN == MarkerTN  || CalEndIN >= InertialN-2  )
     trackedMaker.trackedMakerPosition = trackedMakerPosition ;
%      trackedMaker.time = otherMakersTime ;
     trackedMaker.MarkerSet = otherMakers(1).MarkerSet ;
     N = length(trackedMakerPosition) ;
    for k=1:N
        otherMakers(k).trackedMakerPosition = trackedMakerPosition(:,k) ;
    end
    for k=1:N
        i = N-k+1;
        if ~isnan(trackedMakerPosition(1,i))
            trackedMaker.LastMakerPosition = trackedMakerPosition(:,i);
            trackedMaker.LastMakerTime = otherMakers(i).time ;
            break;
        end
    end
    
    DrawTracedINSVNS( TrackFlag,trackedMaker,InertialPosition,trackedMarkerVelocity,InertialVelocity,INSVNSCalib_VS_k,INSVNSCalibSet ) ;
    
end

if coder.target('MATLAB') && (CalEndVN == MarkerTN  || CalEndIN >= InertialN-2  )
    otherMakersTime = ( 1:MarkerTN )/1;

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
end


function otherMakersEmpty = GetEmpty_otherMakers(otherMakers_k)
otherMakersEmpty = otherMakers_k;
otherMakersEmpty.frequency = NaN;
otherMakersEmpty.Position = NaN(3,1);
otherMakersEmpty.otherMakersN = int32(-1);
%　otherMakers(k).time
%　otherMakers(k).MarkerSet

function otherMakersNew_k = CompensateSecond( otherMakersNew_k,Crw  )
M = otherMakersNew_k.otherMakersN ;

if ~isempty(otherMakersNew_k.Position)
    otherMakersNew_k.Position(:,1:M) = Crw*otherMakersNew_k.Position(:,1:M) ;
end


%% Judge which is the right maker
% 1) 固定运动时间位移判断：只判断位移差长度
% 2）固定运动距离位移判断：同时判断位移差长度和方向

function [ trackedMakerPosition_k_OK,otherMakers_k,TrackFlag,JudgeIndex ] = JudgeMaker...
( otherMakers_k,otherMakers_k_last,k_vision,inertial_k,trackedMakerPosition,InertialPosition,inertial_dT_k_last,...
            makerTrackThreshold )
coder.extrinsic('fprintf');
global  visionFre   IsGetFirstMarker
global InertialData_visual_k  VisionData_inertial_k CalStartIN  CalEndIN

vision_dT_k_last = InertialData_visual_k(inertial_dT_k_last); % 初始值

moveDistance = makerTrackThreshold.moveDistance;
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
        
% M = otherMakers_k.otherMakersN ;
otherMakersPosition_k = otherMakers_k.Position ;


if isnan(otherMakersPosition_k(1))
    return;
end
%% 高度判断
%  将 otherMakers_k 中高度变化大的点直接剔除了
[ otherMakers_k,trackedMakerPosition_k_OK,TrackFlag,INSVNSMarkHC_Min ] = Track_High_Judge...
    ( otherMakers_k,inertial_k,InertialPosition,makerTrackThreshold ) ;
JudgeIndex.INSVNSMarkHC_Min = INSVNSMarkHC_Min  ;
M = otherMakers_k.otherMakersN ;  % 点的个数可能被减少了
if TrackFlag == -1
    return;
end
%% 马克点连续性判断
[ otherMakers_k,dPi_ConJudge ] = ContinuesJudge( otherMakers_k,otherMakers_k_last,trackedMakerPosition,k_vision,makerTrackThreshold );
JudgeIndex.dPi_ConJudge = dPi_ConJudge ;

%% dT 时间段的位移差：只考虑位移矢量大小
% 如果 vision_dT_k_last 没跟踪成功，把时间往前推直到找到跟踪成功的点。但是不能往前推超过 Max_dT 时间。搜索了
% Max_dT时间还没有找到的话，
while vision_dT_k_last>1 && isnan( trackedMakerPosition(1,vision_dT_k_last) ) % trackedMakerPosition(1) 必须已知（不为nan）
    vision_dT_k_last = vision_dT_k_last-1 ;
    inertial_dT_k_last = int32(VisionData_inertial_k(vision_dT_k_last));
    inertial_dT_k_last = max(inertial_dT_k_last,1);
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
            % 第一个马克点搜索的关键：以当前马克点对应的连续点假设为跟踪成功点
            trackedMakerPosition_last_k_dT = otherMakers_k.ContinuesLastPosition(:,i) ; 

            [ trackedMakerPosition_k_OK,TrackFlag,min_dT_k,dPError_dT_xy,dPError_dT_z,dP_Inertial_xyNorm,angleErr_dT_Min ] = Track_dT_Judge...
                ( otherMakers_k,InertialPosition,inertial_k,inertial_dT_k_last,trackedMakerPosition_last_k_dT,makerTrackThreshold );
            JudgeIndex.dPError_dT_xy = dPError_dT_xy ;
            JudgeIndex.dPError_dT_z = dPError_dT_z ;
            JudgeIndex.dP_Inertial_xyNorm = dP_Inertial_xyNorm ;
            JudgeIndex.angleErr_dT_Min = angleErr_dT_Min;
     
            if ~isnan(trackedMakerPosition_k_OK(1)) && IsGetFirstMarker == 0
                IsGetFirstMarker = 1 ;
                
                fprintf( '第一个点搜素成功 time = %0.1f sec, TrackFlag(%d) = %0.1f \n',k_vision/visionFre,k_vision,TrackFlag );
                fprintf(' dPError_dT_xy = %0.3f,  dPError_dT_z = %0.3f \n ',dPError_dT_xy,dPError_dT_z); 
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
        Track_dT_Judge( otherMakers_k,InertialPosition,inertial_k,inertial_dT_k_last,trackedMakerPosition_last_k_dT,makerTrackThreshold );
    JudgeIndex.dPError_dT_xy = dPError_dT_xy ;
    JudgeIndex.dPError_dT_z = dPError_dT_z ;
    JudgeIndex.dP_Inertial_xyNorm = dP_Inertial_xyNorm ;
    JudgeIndex.angleErr_dT_Min = angleErr_dT_Min;
end
if ~isnan(trackedMakerPosition_k_OK(1))    
   return;  % 跟踪OK 
end
otherMakersPosition_k_min = otherMakersPosition_k(:,min_dT_k);
      
%% dS 位移长度段的位移差：同时考虑位移差大小和方向
[ trackedMakerPosition_k_OK,TrackFlag,dPError_dS_xyNorm,angleErr_dS ] = Track_dS_Judge...
    ( InertialPosition,inertial_k,trackedMakerPosition,otherMakersPosition_k_min,TrackFlag,makerTrackThreshold ) ;
JudgeIndex.dPError_dS_xyNorm = dPError_dS_xyNorm  ;
JudgeIndex.angleErr_dS = angleErr_dS  ;
%% 高度判定
% （1.1）惯性目标关节与视觉目标马克点高度差（INSVNSMarkHC） =  INSVNSMarkL *
% cos(thita)。INSVNSMarkL 为常数，在初始时刻刻计算得到。
%  INSVNSMarkHC 计算方法：当前高度-直立时的高度
function [ otherMakers_k,trackedMakerPosition_k_OK,TrackFlag,INSVNSMarkHC_Min ] = Track_High_Judge...
    ( otherMakers_k,inertial_k,InertialPosition,makerTrackThreshold )
 INSMarkH0  = makerTrackThreshold.INSMarkH0 ;
 VNSMarkH0 = makerTrackThreshold.VNSMarkH0;

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
        Position_Old = otherMakers_k.Position;
        otherMakers_k.Position = RemoveOtherMarkers_Position( Position_Old,i-invalid_i,otherMakers_k.otherMakersN );
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

%% 剔除 otherMakers_k.Position 中的第 i 个点

function Position = RemoveOtherMarkers_Position( Position,i,otherMakersN_old )
for k=i:otherMakersN_old-1
    Position(:,k) = Position(:,k+1);
end
Position(:,otherMakersN_old) = NaN(3,1);

%% 马克点判定 4) 寻找惯性运动 dS 长度(大于 moveDistance)，且马克点跟踪成功的是时刻，判定 dP_Inertial 和 dP_Vision：
function [ trackedMakerPosition_k_OK,TrackFlag,dPError_dS_Norm,angleErr_dS ] = Track_dS_Judge...
    ( InertialPosition,inertial_k,trackedMakerPosition,otherMakersPosition_k_min,TrackFlag,makerTrackThreshold )
global InertialData_visual_k

trackedMakerPosition_k_OK=  NaN ;
%% dS 位移长度段的位移差：同时考虑位移差大小和方向
% find the point which moved moveDistance
moveDistance = makerTrackThreshold.moveDistance;
dS_Inertial_last_k = SearchDistanceK( InertialPosition,inertial_k,moveDistance,trackedMakerPosition ) ;  
if isnan(dS_Inertial_last_k)
    % 找不到运动大于 dS 长度且trackedMakerPosition中跟踪到的点
%     fprintf('找不到运动固定长度的点，不进行验证<2>，跟踪失败。\n');
    dPError_dS_Norm = -0.2 ;    % 给负值表示没有找到
    angleErr_dS = -10*pi/180;
    TrackFlag = -1.4 ;
    return; 
end
dS_Vision_last_k = InertialData_visual_k(dS_Inertial_last_k)  ;

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
    = Track_dT_Judge( otherMakers_k,InertialPosition,inertial_k,inertial_dT_k_last,trackedMakerPosition_last_k_dT,makerTrackThreshold )

% global InertialData
trackedMakerPosition_k_OK = NaN ;
otherMakersPosition_k = otherMakers_k.Position ;
TrackFlag = 0;

M = size(otherMakersPosition_k,2) ;
dP_Inertial = InertialPosition(:,inertial_k) - InertialPosition(:,inertial_dT_k_last);
dP_Inertial_xy = dP_Inertial(1:2);
dP_Inertial_z = dP_Inertial(3);  % 向下为正
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
    dPError_dT_z(i) = dP_Inertial_z-dP_Vision(3,i);  
    % 低头时，dP_Inertial_z和dP_Vision(3,i)都为正，dP_Vision(3,i)的模更大，dPError_dT_z(i)为负
    
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
       if HighMoveErr>MaxHighMoveErrRate(1) && HighMoveErr < MaxHighMoveErrRate(2)
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





%% 查找惯性k时刻前运动了大于 dS 距离的最近的点
% 且该点trackedMakerPosition跟踪成功
function dS_Inertial_last_k = SearchDistanceK( InertialPosition,kCurrent,dS,trackedMakerPosition )
global inertialFre InertialData_visual_k
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
%         kCurrent_Vision = InertialK_to_VisionK(kCurrent-i);
        kCurrent_Vision = InertialData_visual_k(kCurrent-i);
        if ~isnan(trackedMakerPosition(1,kCurrent_Vision))            
            % 距离满足，且 trackedMakerPosition 跟踪成功
            dS_Inertial_last_k = double(kCurrent-i) ;
            break;
        end
    end
 %   waitbar(i/kCurrent);
end
%close(wh)
disp('')

