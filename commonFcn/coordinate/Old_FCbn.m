%% 姿态角->方向余弦矩阵
%　此函数要求：姿态定义在 东北天坐标系 NET：旋转顺序为 Rz(yaw)->Rx(pitch)->Ry(roll)
% posture = [ pitch roll yaw ]
function [Cbn]=FCbn(posture)
% 俯仰、横滚、航向 rad
format long
% st=posture(1);
% r=posture(2);
% fai=posture(3);
% 
% t11=cos(r)*cos(fai)-sin(r)*sin(st)*sin(fai);
% t12=cos(r)*sin(fai)+sin(r)*sin(st)*cos(fai);
% t13=-sin(r)*cos(st);
% t21=-cos(st)*sin(fai);
% t22=cos(st)*cos(fai);
% t23=sin(st);
% t31=sin(r)*cos(fai)+cos(r)*sin(st)*sin(fai);
% t32=sin(r)*sin(fai)-cos(r)*sin(st)*cos(fai);
% t33=cos(r)*cos(st);
% cnb=[t11,t12,t13;t21,t22,t23;t31,t32,t33];
% Cbn=cnb';

pitch=posture(1);
roll=posture(2);
yaw=posture(3);
Cbn(1, 1) = cos(roll) * cos(yaw) - sin(roll) * sin(pitch) * sin(yaw);
Cbn(1, 2) = -cos(pitch) * sin(yaw);
Cbn(1, 3) = sin(roll) * cos(yaw) + cos(roll) * sin(pitch) * sin(yaw);     
Cbn(2, 1) = cos(roll) * sin(yaw) + sin(roll) * sin(pitch) * cos(yaw);
Cbn(2, 2) = cos(pitch) * cos(yaw);
Cbn(2, 3) = sin(roll) * sin(yaw) - cos(roll) * sin(pitch) * cos(yaw);     
Cbn(3, 1) = -sin(roll) * cos(pitch); 
Cbn(3, 2) = sin(pitch);
Cbn(3, 3) = cos(roll) * cos(pitch);
% Cnb=Cbn';
