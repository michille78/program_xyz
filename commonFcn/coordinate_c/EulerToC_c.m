%% xyz 2015.4.5
%  EulerToC_Multi  新版 ： 可转化为 C++

%   euler: N*3   euler(k,:) = [eu_x,eu_y,eu_z] 
% store order of euler angle: the rotate order
% positive:anticlockwise rotate 
% euler angle scope: [-pi,pi]
% unit:rad 
% rotateDirection: anticlockwise(1) or clockwise(-1)  111


function C =  EulerToC_c( euler,rotateOrder,rotateDirection )
coder.inline('never');

Nframes = size( euler,1 );
C = zeros( Nframes,3,3 );
for k=1:Nframes
   C( k,:,: )  = OneEulerToC_c( euler(k,:),rotateOrder,rotateDirection );
end


function C =  OneEulerToC_c( euler,rotateOrder,rotateDirection )
coder.inline('never');

for i=1:3
    euler(i) = euler(i)*rotateDirection(i) ;
end

switch rotateOrder
    case 'ZYX' 
        C = RotateX(euler(3)) * RotateY(euler(2)) * RotateZ(euler(1)) ;
    case 'ZXY'
        C = RotateY(euler(3)) * RotateX(euler(2)) * RotateZ(euler(1)) ;
    case 'XYZ'
        C = RotateZ(euler(3)) * RotateY(euler(2)) * RotateX(euler(1)) ;
    case 'XZY'
        C = RotateY(euler(3)) * RotateZ(euler(2)) * RotateX(euler(1)) ;
    case 'YXZ'
        C = RotateZ(euler(3)) * RotateX(euler(2)) * RotateY(euler(1)) ;
    case 'YZX'
        C = RotateX(euler(3)) * RotateZ(euler(2)) * RotateY(euler(1)) ;
end