
%% 根据 BodyDirection 求姿态
% 符号有待验证！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！
%   BodyDirection  人朝北为 [1 0 0]
function Cr_r1 = BodyDirection2Cr_r1( BodyDirection )
coder.inline('never');

V = [1 0 0];   % 正北
thita = acos( V*BodyDirection / normest(BodyDirection)   );
% 通过叉乘可判断角度方向
temp = cross(V,BodyDirection) ;  % 从 V 到 BodyDirection 逆时针180°以内时，temp朝上，temp(3)<0
if temp(3)>0
        % 从 V 到 BodyDirection 逆时针转动超过180°
      thita = -thita ;
end
    
Cr_r1 = RotateZ( thita ) ;