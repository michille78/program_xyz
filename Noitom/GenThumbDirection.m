%% xyz 2015.5.27

%% （右手）根据拇指的角度状态输出拇指坐标轴 各个Pose时， 在世界坐标系下的指向
% 拇指的骨骼系为“左上前”，旋转顺序为“YZX”
% w:当地北东地系
% r:与w系仅相差航向信息的水平系（x指向人的前方）
% p：特定pose下手掌系
%%% Input:
% rotateAngle：拇指的“YZX”旋转角度,单位：度  [3*1] 
% zd： 人的前方在世界系下的表达（r系x轴在w系下表达）  [3*1]
% RightOrLeft： 左手（ 'L'）， 右手 （'R'）
% pose"：姿势 'T' 'A'  'S'
function  bone_w = GenThumbDirection( rotateAngle,zd,RightOrLeft,pose )

if strcmp(RightOrLeft,'L')
    rotateAngle(1:2) = -rotateAngle(1:2);
end
rotateOrder = 'YZX';
rotateAngle = rotateAngle*pi/180 ;
%% 当前pose下手掌系 p  -> 拇指系
Cp_ThumbBone = Euler2C( rotateAngle,rotateOrder) ;
%% 无航向的世界系 r -> 当前pose下手掌系 p
switch pose
    case 'T' 
        Crp = eye(3);
    case 'A'
        if strcmp(RightOrLeft,'L')
            Crp = RotateX(-pi/2);
        else
            Crp = RotateX(pi/2);
        end
    case 'S'
        if strcmp(RightOrLeft,'L')
            Crp = RotateZ(pi/2);
        else
            Crp = RotateX(-pi/2);
        end
    otherwise
        Crp = eye(3);
end
%% Crw
rx_w = zd ;                 % zd = [0; 1; 0];
rz_w = [0;0;1];
ry_w = cross( rx_w,rz_w );
Crw = [ rx_w,ry_w,rz_w ];  % 

%% 默认值 ：人朝东  zd = [0; 1; 0]; 
% Crw =
%      0     1     0
%      1     0     0
%      0     0     1
%%
Cw_ThumbBone =  Cp_ThumbBone * Crp * Crw' ;


bone_w = Cw_ThumbBone' * eye(3);
