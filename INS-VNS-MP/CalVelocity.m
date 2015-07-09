%% xyz 2015.6.1

%% 计算某个点的速度
%% 得到 第 k 个点的位置后，计算的是第 k_calV 个点的速度（ k_calV = data_k-dN_CalV ;）
% dT_CalV： 速度计算步长
%  V(k) = (X(k+dN_CalV) - X(k-dN_CalV)) / (dT_CalV*2)

% fre： 频率
% dT_CalV： 速度计算的步长时间
% MinXYVNorm_CalAngle： 计算xy速度方向要求的最小xy速度模值
% CalVFlag： 速度计算方法标志

%% Velocity_k
% Velocity_k(1:3,1) : xyz三维的速度
% Velocity_k(4,1) : xy平面速度的模
% Velocity_k(5,1) : xy平面速度与正前方之间的夹角（模大于0.2m/s时才计算）

%% DataStore 数据存储格式
% DataStore.IsLoopStore =0/1  是否循环存储
% DataStore.BN   BufferN    缓存大小
% DataStore.EN   EndN       结束序号
% DataStore.SN   StartN     起始序号

function [ Velocity_k,k_calV ] = CalVelocity( Position,data_k,fre,dT_CalV,MinXYVNorm_CalAngle,CalVFlag )

% 速度计算的步长个数

dT_CalV = min(dT_CalV,0.2);
dN_CalV = fix(dT_CalV*fre) ;
dN_CalV = max(dN_CalV,1);

angleXY = NaN ;
Velocity_k = NaN(5,1) ;

%% 计算速度： 
% 计算第 k_calV 个数的速度：用 [ k_calV-dN_CalV, k_calV+dN_CalV  ] 的一段数据
% 得到 data_k 数据后，计算第 data_k-dN_CalV 个数的速度
k_calV = data_k-dN_CalV ;

if k_calV-dN_CalV<1 || k_calV+dN_CalV>length(Position)    
    return; 
end


%% xyz三维的速度
switch CalVFlag
    case 1
        %% 直接采用端区间端点起斜率
        Position1 = Position( :,k_calV+dN_CalV ) ;
        Position2 = Position( :,k_calV-dN_CalV ) ;
        
    case 2
        %% 采用 [k_calV-dN_CalV:k_calV-1 ]和[k_calV+1:k_calV+dN_CalV]的均值求斜率
        Position1_A = Position( :,k_calV+1:k_calV+dN_CalV ) ;
        Position2_A = Position( :,k_calV-dN_CalV:k_calV-1 ) ;
        Position1 = mean(Position1_A,2);
        Position2 = mean(Position2_A,2);
end

if isnan( Position1(1) ) || isnan( Position2(1) )
    Velocity_k = NaN(5,1) ;
    return;  % 计算速度的数据中有跟踪失败的点 
end
Velocity_k = ( Position1 - Position2 ) / (dT_CalV*2) ;
        

% 第四行存储xy平面速度的模
trackedMarkerVelocity_xyNorm = normest(Velocity_k(1:3)) ; 

% 第五行存储xy平面速度与正前方之间的夹角（模大于0.2m/s时才计算）
if trackedMarkerVelocity_xyNorm >  MinXYVNorm_CalAngle 
   temp =  [0 1]*Velocity_k(1:2) / trackedMarkerVelocity_xyNorm ;
   angleXY = acos(temp);
   if Velocity_k(1)>0
       angleXY = -angleXY ;
   end
end
Velocity_k = [ Velocity_k; trackedMarkerVelocity_xyNorm;angleXY ]; 