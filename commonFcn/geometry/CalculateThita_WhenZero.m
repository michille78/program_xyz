%% xyz 2015.4.10

% 当磁罗盘数据为正圆时， ellipsePolyToStand 输出的 thita=0
% 采用 刚体最小阶惯性矩法 计算 thita

% eStand_Fitting.cx  .cy  .thita  .a  .b  .e  .f
%  H_n_xy (N*2)

%% ref
% SKVORTZOV V Y, LEE H-K, BANG S, et al. Application of electronic compass 
% for mobile robot in an indoor environment; proceedings of the Robotics and Automation, 
% 2007 IEEE International Conference on, F, 2007 [C]. IEEE.

function [ eStand_Fitting,ThitaFlag ] = CalculateThita_WhenZero( eStand_Fitting,H_n_xy )

cx = eStand_Fitting.cx;
cy = eStand_Fitting.cy;
% a = eStand_Fitting.a;
% b = eStand_Fitting.b;
% e = eStand_Fitting.e;
% f = eStand_Fitting.f;
thita = eStand_Fitting.thita;

if thita(1)~=0
    ThitaFlag = 'polynomial';
    return;
end

ThitaFlag = 'inertial moment';

N = length( H_n_xy );
H_n_xy_New = H_n_xy - repmat( [ cx, cy],N,1 );
H_n_xy_New_s = H_n_xy_New.^2 ;

U = mean( H_n_xy_New_s,1 );
Uxx = U(1);
Uyy = U(2);

Uxy  = H_n_xy_New(:,1)'*H_n_xy_New(:,2) /N   ;

thita_new = 0.5*atan( 2*Uxy/(Uxx-Uyy) );
eStand_Fitting.thita = thita_new ;

