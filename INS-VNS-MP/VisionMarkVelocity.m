%% xyz 2015.6.1

%% 根据跟踪成功的马克点提取惯性视觉坐标系标定数据
% 每跟踪成功一个点判断这个点是否可以更新坐标系标定数据

function [ trackedMarkerVelocity_k,k_calV ] = VisionMarkVelocity( trackedMakerPosition,vision_k )

global  visionFre INSVNSCalibSet

dT_CalV_Calib = INSVNSCalibSet.dT_CalV_Calib ; % 计算速度时间步长（标定位移数据选择）
dN_CalV_Calib = fix(dT_CalV_Calib*visionFre) ;
dN_CalV_Calib = max(dN_CalV_Calib,2);
dN_CalV_Calib = min(dN_CalV_Calib,7);

angleXY = NaN ;
trackedMarkerVelocity_k = NaN(5,1) ;

% 计算速度： 
% 计算第 k_calV 个数的速度：用 [ k_calV-dN_CalV_Calib, k_calV+dN_CalV_Calib  ] 的一段数据
% 得到 vision_k 数据后，计算第 vision_k-dN_CalV_Calib 个数的速度
k_calV = vision_k-dN_CalV_Calib ;
if k_calV-dN_CalV_Calib<1 || k_calV+dN_CalV_Calib>length(trackedMakerPosition)    
    return; 
end

trackedMakerPosition1 = trackedMakerPosition( :,k_calV+dN_CalV_Calib ) ;
trackedMakerPosition2 = trackedMakerPosition( :,k_calV-dN_CalV_Calib ) ;
if isnan( trackedMakerPosition1(1) ) || isnan( trackedMakerPosition2(1) )
    trackedMarkerVelocity_k = NaN(5,1) ;
    return;  % 计算速度的数据中有跟踪失败的点 
end

trackedMarkerVelocity_k = ( trackedMakerPosition1 - trackedMakerPosition2 ) / (dT_CalV_Calib*2) ;
% 第四行存储xy平面速度的模
trackedMarkerVelocity_xyNorm = normest(trackedMarkerVelocity_k(1:2)) ; 

% 第五行存储xy平面速度与正前方之间的夹角（模大于0.2m/s时才计算）
if trackedMarkerVelocity_xyNorm >  INSVNSCalibSet.MinXYVNorm_CalAngle 
   temp =  [0 1]*trackedMarkerVelocity_k(1:2) / trackedMarkerVelocity_xyNorm ;
   angleXY = acos(temp);
   if trackedMarkerVelocity_k(1)>0
       angleXY = -angleXY ;
   end
end
trackedMarkerVelocity_k = [ trackedMarkerVelocity_k; trackedMarkerVelocity_xyNorm;angleXY ]; 