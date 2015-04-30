%% xyz 2015.4.23
% 从四元数中提取角度和单位矢量
% angle： -pi ~ pi  [1*N]
% vectorNormed： [3*N]  单位矢量，方向一致

function [ QAngle,QVectorNormed] = GetQAngle( Q )


Q = Make_Const_N(Q,4);
Q_NormVector = GetNormVectorQ( Q );
QVectorNormed = Q_NormVector( 2:4,: );

N = size(Q,2);
QAngle =  zeros(1,N);
for k=1:N
    QAngle(k) = acot( Q_NormVector )*2 ;
end
