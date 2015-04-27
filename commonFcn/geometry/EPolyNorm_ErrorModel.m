%% xyz 2015 3.20
% 将 ePoly 转换为与 磁罗盘误差模型 对应的标准 ePolyNormalized
% H_Norm 为真实的磁场矢量模大小  （椭圆时 H_Norm 为旋转平面内的次啊很难过矢量模，椭球时为3维空间模）

function [ ePolyNormalized,rate ] = EPolyNorm_ErrorModel( ePoly,H_Norm)

[ A,B,C,D,E,F ] = deal( ePoly(1),ePoly(2),ePoly(3),ePoly(4),ePoly(5),ePoly(6) ) ;
K = [ A  B/2; B/2  C ];
b  = ((-2*K)')\[D;E]; 

rate = H_Norm^2 / ( b'*K*b-F ) ;
ePolyNormalized = ePoly*rate ;