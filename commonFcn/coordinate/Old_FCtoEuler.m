%% xyz  2015.3.9

% direction cosine matrix  to Euler angle

%       euler = [eu_x,eu_y,eu_z] 
% store order of euler angle: the rotate order
% positive:anticlockwise rotate 
% euler angle scope: [-pi,pi]
% unit:rad 

function euler = FCtoEuler( C,rotateOrder )

if ~exist('rotateOrder','var')
    rotateOrder = 'ZYX';
end

switch rotateOrder
    case 'ZYX' 
        euler1 = atan2( C(1,2),C(1,1) ) ;
        euler2 = atan2( -C(1,3),sqrt( C(3,2)^2+C(3,3)^2 ) ) ;
        euler3 = atan2( C(2,3),C(3,3) ) ;
    case 'ZXY'
        euler1 = atan2( C(2,1),C(2,2) ) ;
        euler2 = atan2( C(2,3),sqrt( C(2,1)^2+C(2,2)^2 ) ) ;
        euler3 = -atan2( C(1,3),C(3,3) ) ;
    case 'XYZ'
    case 'XZY'
    case 'YXZ'
        euler1 = atan2( C(3,1),C(3,3) ) ;
        euler2 = atan2( -C(3,2),sqrt( C(1,2)^2+C(2,2)^2 ) ) ;
        euler3 = atan2( C(1,2),C(2,2) ) ;        
    case 'YZX'  
end
euler = [ euler1; euler2; euler3 ];
