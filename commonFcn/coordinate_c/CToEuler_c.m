%% xyz  2015.4.5

% direction cosine matrix  to Euler angle  for multi C
% C: [N*3*3]
%       euler = [eu_x,eu_y,eu_z]  ( Nframes*3 )
% store order of euler angle: the rotate order
% positive:anticlockwise rotate 
% the first and third euler angles scope: [-pi,pi]
% the second euler angles scope: [-pi/2,pi/2]
% unit:rad 
% rotateDirection: anticlockwise(1) or clockwise(-1)  111

function euler = CToEuler_c( C,rotateOrder,rotateDirection )
coder.inline('never');

Nframes = size( C,1 );
euler = zeros( Nframes,3 );
for k=1:Nframes
    C_K = permute( C(k,:,:),[2,3,1] );
    euler(k,:) = OneCToEuler_c( C_K,rotateOrder,rotateDirection );
end


function euler = OneCToEuler_c( C,rotateOrder,rotateDirection )
coder.inline('never');

switch rotateOrder
    case 'ZYX' 
        euler1 = atan2( C(1,2),C(1,1) ) ;
        euler2 = asin( -C(1,3) ) ;
        % euler2 = atan2( -C(1,3),sqrt( C(3,2)^2+C(3,3)^2 ) ) ;
        euler3 = atan2( C(2,3),C(3,3) ) ;
    case 'ZXY'
        euler1 = atan2( -C(2,1),C(2,2) ) ;
        euler2 = asin( C(2,3) ) ;
        % euler2 = atan2( C(2,3),sqrt( C(2,1)^2+C(2,2)^2 ) ) ;
        euler3 = atan2( -C(1,3),C(3,3) ) ;
    case 'XYZ'
        euler1 = atan2( -C(3,2),C(3,3) ) ;
        euler2 = asin(C(3,1)) ;
        euler3 = atan2( -C(2,1),C(1,1) ) ;
    case 'XZY'
        euler1 =  atan2(C(2,3),C(2,2)) ;
        euler2 =  asin(-C(2,1)) ;
        euler3 =  atan2(C(3,1),C(1,1)) ;
    case 'YXZ'
        euler1 = atan2( C(3,1),C(3,3) ) ;
        euler2 = asin( -C(3,2) ) ;
        % euler2 = atan2( -C(3,2),sqrt( C(1,2)^2+C(2,2)^2 ) ) ;
        euler3 = atan2( C(1,2),C(2,2) ) ;        
    case 'YZX'  
        euler1 = atan2(-C(1,3),C(1,1))  ;
        euler2 = asin(C(1,2)) ;
        euler3 = atan2(-C(3,2),C(2,2)) ;
end
euler = [ euler1; euler2; euler3 ];

for i=1:3
    euler(i) = euler(i)*rotateDirection(i) ;
end
