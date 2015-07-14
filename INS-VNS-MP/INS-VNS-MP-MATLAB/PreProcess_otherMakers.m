



%% 视觉位置预处理
% 1）将视觉数据  从视觉世界坐标系 转到 北东地坐标系
function otherMakers_k = PreProcess_otherMakers( otherMakers_k  )
coder.inline('never');

%% 先转到北东地的同名坐标系
Cv_r1 = [ 0 0 1; -1 0 0; 0 -1 0 ];
% dbstop in BodyDirection2Cr_r1
% Cr_r1 = BodyDirection2Cr_r1( BodyDirection );   %   （法1）    要求人朝视觉标定标定的世界坐标系进行对准
% Cvr = Cr_r1' * Cv_r1 ;
Cvr = Cv_r1 ;         %   （法2）    要求视觉世界坐标系的Z轴朝向正北




if ~isempty(otherMakers_k.Position)
    otherMakers_k.Position = Cvr*otherMakers_k.Position;    
end




