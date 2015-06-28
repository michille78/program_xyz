
%% 阈值参数设置
function [ makerTrackThreshold,INSVNSCalibSet ] = SetConstParameters( visionFre )


moveTime = 2 ;          % sec 轨迹连续判断时间步长
moveDistance = 0.5 ;      % m   轨迹连续判断位移步长  （经验值建议0.4m-0.7m）
MaxMoveSpeed = 6 ; % m/s  马克点运动允许的最大速度，超过这个速度则认为不连续
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

makerTrackThreshold.MaxHighMoveErrRate = [ -0.3  0.5 ] ;  %  高度方向变化大时，若变化比例小于这个值，直接认定跟踪OK
        % 发现高度方向的误差非常
makerTrackThreshold.BigHighMove = 0.18 ;         % m 大于这个值则认为高度方向变化大
%% 坐标系标定参数
INSVNSCalibSet.Min_xyNorm_Calib = 0.3 ; % m  用于标定的数据的最小运动位移长度
INSVNSCalibSet.MaxTime_Calib = 2  ;  % sec  用于标定的数据的最长时间
INSVNSCalibSet.MaxVXY_DirectionChange_Calib = 30*pi/180 ;     % ° XY平面速度方向变化最大范围
INSVNSCalibSet.MaxVZ_Calib = 0.1 ;     % m/s Z方向速度最大绝对值
INSVNSCalibSet.MinVXY_Calib = 0.2;   	% m/s XY 平面速度模最小绝对值
INSVNSCalibSet.angleUniformityErr = 10*pi/180 ; % ° 位移矢量方向均匀性误差
% 速度计算
INSVNSCalibSet.dT_CalV_Calib = 0.15 ; % 计算速度时间步长（标定位移数据选择）  0.1 
INSVNSCalibSet.MinXYVNorm_CalAngle = 0.1 ;  %  m/s xy速度模大于这个值才计算速度的反向

makerTrackThreshold.INSMarkH0 = NaN ;
makerTrackThreshold.VNSMarkH0 = NaN ;