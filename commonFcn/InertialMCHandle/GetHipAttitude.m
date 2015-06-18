%% xyz  2015.6.8
%% 从 Hip 的 骨骼NED全局 四元数得到 姿态角

% HipQ： Hip 骨骼 在 NED 世界坐标系 下的姿态  [4*N]
% HipAttitude： [ yaw; pitch; roll ]  rad

function HipAttitude = GetHipAttitude( HipQ )
N = size( HipQ,2 );
HipAttitude = zeros( 3,N );
for k=1:N
    HipAttitude(:,k) = GetHipAttitude_One( HipQ(:,k) ) ;
end


function Attitude = GetHipAttitude_One( HipQuaternion_k )

HipQuaternion_k = Qinv( HipQuaternion_k ) ; % 四元数定义由 顺时针 改为 逆时针。

CHip_k = Q2C(HipQuaternion_k);  % bone 到 bvhGlobal        
Attitude = C2Euler( RotateX(pi/2)*CHip_k ,'ZYX' ) ;

