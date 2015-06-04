%% xyz 2015 儿童节特供

%% 根据 trackedMarkerVelocity 自动选择用于标定惯性和视觉坐标系的数据
% INSVNSCalib_VS_k ： [2*N]
    % INSVNSCalib_VS_k(1,k)为某个位移的起始，INSVNSCalib_VS_k(2,k)为某个位移的结束
function [ INSVNSCalib_VS_k,IsCalibDataEnough,dX_Vision ] = SearchCalibData...
            ( INSVNSCalib_VS_k,trackedMarkerVelocity,trackedMakerPosition,vision_k )

global  visionFre
IsCalibDataEnough = 0;
dX_Vision = [];
if isnan( trackedMarkerVelocity(1,vision_k) ) || isnan( trackedMakerPosition(1,vision_k) )
    return;
end
%% 阈值参数设置
global INSVNSCalibSet
MaxTime_Calib = INSVNSCalibSet.MaxTime_Calib ;  % sec  用于标定的数据的最长时间
MaxVXY_DirectionChange_Calib = INSVNSCalibSet.MaxVXY_DirectionChange_Calib ;     % ° XY平面速度方向变化最大范围

MaxN_Calib = fix(MaxTime_Calib*visionFre) ;

%% 从上一段位移差数据之后开始搜索
Calib_N_Last = size(INSVNSCalib_VS_k,2);   % 搜索成功的标定位移数据个数
if Calib_N_Last>0 
    LastEnd_k = INSVNSCalib_VS_k(2,Calib_N_Last) ;
else
    LastEnd_k = 0 ;  % 第一次搜索
end
% 从 LastEnd_k+1 搜索到 vision_k
search_k = vision_k ;
%% 搜索末尾点

search_k_end = NaN;
while search_k > LastEnd_k   
    IsCalibDataVelocityOK = CalibDataVelocityJudge( trackedMarkerVelocity(:,vision_k) ) ;
    if IsCalibDataVelocityOK == 1
        search_k_end = search_k ; % 从最新的数据开始搜索，得到第一个OK的点作为末尾点
%         fprintf('search_k_end = %d \n ',search_k_end)
        break; 
    end
    search_k = search_k-1 ;
end
if isnan(search_k_end)
    return; % 搜索失败
end
%% 搜索起点
search_k_start = NaN ;
while search_k > LastEnd_k  &&  (search_k_end-search_k+1)<MaxN_Calib
    IsCalibDataVelocityOK = CalibDataVelocityJudge( trackedMarkerVelocity(:,vision_k) ) ;
    if IsCalibDataVelocityOK == 1
        search_k_start_temp = search_k+1 ; % 得到速度大小满足条件的 起始点，再判断位移长度
        [ IsCalibDataDistanceOK,dX_xyNorm_VS ] = CalibDataDistanceJudge( trackedMakerPosition,search_k_start_temp,search_k_end ) ;
        
        if IsCalibDataDistanceOK==1
            search_k_start = search_k_start_temp;
            break; 
        end         
    end
    search_k = search_k-1 ;
end
if isnan(search_k_start)
    return; % 搜索失败
end
%% 判断这段区间速度的角度变化是够小

VelocityDirection = trackedMarkerVelocity( 5,search_k_start:search_k_end );  % 速度方向
VelocityDirectionRange = max(VelocityDirection) - min(VelocityDirection);
if VelocityDirectionRange > MaxVXY_DirectionChange_Calib
    return; % 搜索失败  速度方向变化范围太大
end

%% 搜索一个位移成功
INSVNSCalib_VS_k = [ INSVNSCalib_VS_k [search_k_start;search_k_end] ];
%% 判断当前所有搜索的位移是否满足均匀特性
[ IsCalibDataEnough,dX_Vision ] = JudgeIsCalibDataEnough( INSVNSCalib_VS_k,trackedMakerPosition );

searchT = (search_k_end-search_k_start)/visionFre ;
fprintf( '\n 第%d段位移：[%d  %d]sec，  \n 角度范围 = %0.2f °，位移长度 = %0.2f m，\n   时间=%0.2f sec ，平均速度： %0.2f m/s \n',...
    Calib_N_Last+1,search_k_start/visionFre,search_k_end/visionFre,VelocityDirectionRange*180/pi,dX_xyNorm_VS,searchT,dX_xyNorm_VS/searchT );

%% 判断搜索得到的位移数据够不够多，是否已经均匀分布
function [ IsCalibDataVelocityOK,dX_Vision ] = JudgeIsCalibDataEnough( INSVNSCalib_VS_k,trackedMakerPosition )
global INSVNSCalibSet
angleUniformityErr = INSVNSCalibSet.angleUniformityErr ;

M = size( INSVNSCalib_VS_k,2 );
dX_Vision = zeros(3,M);
dX_Angle = zeros(1,M);
HaveBiggerData = 0;
HaveSmallerData = 0;
IsCalibDataVelocityOK = 0;
for k=1:M
    dX_Vision(:,k) = trackedMakerPosition(:,INSVNSCalib_VS_k(2,k)) - trackedMakerPosition(:,INSVNSCalib_VS_k(1,k))  ;  % 从 开始 指向 结束
    %% 只考虑平面内的位移
    dX_Vision(3,k) = 0; 
    %% 根据所有位移矢量与第一个矢量的夹角判断是否分布均匀
    angle = acos( dX_Vision(:,1)'*dX_Vision(:,k)/normest(dX_Vision(:,1))/normest(dX_Vision(:,k)) );
    % 通过叉乘可判断角度方向
    if cross(dX_Vision(:,1),dX_Vision(:,k))<0
        % 从 dX_Vision(:,1) 到 dX_Vision(:,k) 逆时针转动超过180°
        angle = -angle ;
    end
    
    % 以dX_Vision(:,1)为中心，对所有的位移矢量，若与dX_Vision(:,1)夹角大于90°则反号
    if angle > pi/2
        angle = angle-pi ;
    end
    if angle < -pi/2
        angle = angle+pi ;
    end    
    
    dX_Angle(k) = angle ;
    % 在 [60-angleUniformityErr,60+angleUniformityErr] 和
    % [-60-angleUniformityErr,-60+angleUniformityErr] 范围内均存在位移矢量时判定分布均匀
    if dX_Angle(k) > pi/3-angleUniformityErr && dX_Angle(k) < pi/3+angleUniformityErr
       HaveBiggerData = 1 ; 
    end
    if dX_Angle(k) > -pi/3-angleUniformityErr && dX_Angle(k) < -pi/3+angleUniformityErr
       HaveSmallerData = 1 ; 
    end
    if HaveSmallerData==1 && HaveBiggerData==1
       %% 判定位移矢量满足分布均匀条件
       IsCalibDataVelocityOK = 1 ;
    end
end
%% 均匀性条件满足时将所有的位移矢量绘制出来
if IsCalibDataVelocityOK==1
   figure('name','dX_Vision') 
   hold on
   for k=1:M
       if k==1
            plot( [-dX_Vision(1,k) dX_Vision(1,k)],[-dX_Vision(2,k) dX_Vision(2,k)],'r' ); 
            hold on 
            plot(dX_Vision(1,k),dX_Vision(2,k),'.r')
       else
           plot( [-dX_Vision(1,k) dX_Vision(1,k)],[-dX_Vision(2,k) dX_Vision(2,k)] ); 
           hold on 
           plot(dX_Vision(1,k),dX_Vision(2,k),'.k')
       end
   end
   dX_Angle_Degree = dX_Angle*180/pi;
end


%% 判断从 search_k_start 到 search_k_end 的这一段位移长度是否够长
function [ IsCalibDataDistanceOK,dX_xyNorm_VS ] = CalibDataDistanceJudge( trackedMakerPosition,search_k_start,search_k_end )
%% 阈值参数设置
global INSVNSCalibSet
Min_xyNorm_Calib = INSVNSCalibSet.Min_xyNorm_Calib ; % m  用于标定的数据的最小运动位移长度

trackedMakerPosition_start = trackedMakerPosition( :,search_k_start );
trackedMakerPosition_end = trackedMakerPosition( :,search_k_end );
dX_VS = trackedMakerPosition_end-trackedMakerPosition_start ;
dX_xy_VS = dX_VS;
dX_xy_VS(3) = 0 ;
dX_xyNorm_VS = normest(dX_xy_VS);
if dX_xyNorm_VS > Min_xyNorm_Calib
    IsCalibDataDistanceOK = 1 ;
else
    % 位移长度太短
    IsCalibDataDistanceOK = 0 ;
end

%% 判断从 search_k_start 到 search_k_end 的这一段速度是否满足条件
% 1） 速度z模小于 MaxVZ_Calib
% 2） 速度xy模大于 MinVXY_Calib
function IsCalibDataVelocityOK = CalibDataVelocityJudge( trackedMarkerVelocity_k )
%% 阈值参数设置
global INSVNSCalibSet
MaxVZ_Calib = INSVNSCalibSet.MaxVZ_Calib;     % m/s Z方向速度最大绝对值
MinVXY_Calib = INSVNSCalibSet.MinVXY_Calib ;   	% m/s XY 平面速度模最小绝对值

IsCalibDataVelocityOK = 0 ;
if isnan(trackedMarkerVelocity_k(1))
   return; 
end
if abs(trackedMarkerVelocity_k(3)) > MaxVZ_Calib
    return;
end
if trackedMarkerVelocity_k(4) < MinVXY_Calib
    return;    
end
IsCalibDataVelocityOK = 1 ;

