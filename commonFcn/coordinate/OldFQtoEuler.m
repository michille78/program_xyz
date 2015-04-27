%% xyz 2015.3.6
% 基于四元数到ZYX欧拉角的转换函数为基础。求四元数到其他欧拉角的转换，通过预先将四元数变形即可。

% Euler = [ afa,beita,gama ] 所有欧拉角存储顺序：旋转顺序。 范围均为 [ -pi,pi ]
function Euler = FQtoEuler(Q,rotateOrder)

if ~exist('rotateOrder','var')
    rotateOrder = 'ZYX';
end

qs = Q(1) ;
qx = Q(2) ;
qy = Q(3) ;
qz = Q(4) ;
switch rotateOrder
    case 'ZYX' 
        Q_New = Q ;  % 输出NED姿态角 [ yaw,pitch,roll ]
    case 'ZXY'
        Q_New = [ qs,qy,qx,qz ];        
    case 'XYZ'
        Q_New = [ qs,qz,qx,qy ];   
    case 'XZY'
        Q_New = [ qs,qz,qy,qx ];   
    case 'YXZ'
        Q_New = [ qs,qy,qz,qx ];   
    case 'YZX'
        Q_New = [ qs,qy,qz,qx ];   
end
Euler = FQtoZYX_Euler(Q_New) ;


%% 四元数Q到ZYX欧拉角
% Euler=[ angZ,angY,angX ] 欧拉角存储顺序：旋转顺序
function Euler = FQtoZYX_Euler(Q)
format long
Cnb = FQtoCnb(Q) ;

angX = atan2( Cnb(2,3),Cnb(3,3) ) ;
% angY = asin( -Cnb(1,3) ) ;
% 另一种angY的解法
angY = atan2(-Cnb(1,3),sqrt(Cnb(3,2)^2+Cnb(3,3)^2)) ;
angZ = atan2(Cnb(1,2),Cnb(1,1)) ;

Euler=[ angZ;angY;angX ] ;  % 按旋转顺序存储
