%% xyz  2015.4.9

% direction cosine matrix  to Euler angle
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Input
% C: [ 3*3*N ]
% rotateOrder:  XYZ  XZY  ...
% rotateDirection: anticlockwise(1) or clockwise(-1)  [1,1,1]
% CosBeitaSign: 1 / -1 . 
%       CosBeitaSign = 1 : cos(beita)>0, beita [ -pi/2,pi/2 ]
%       CosBeitaSign = -1 : cos(beita)<0, beita [ -pi,-pi/2 ] or [ pi/2,pi ]
%%%  NOTE: There are two kinds of Euler Angles applied to the C. 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Output
% euler:  [3*N];   euler(:,k) = [eu_1,eu_2,eu_3];   rad   
%       store order of euler angle: the rotate order
%       eu_1,eu_3: [-pi,pi]
%       eu_2:   CosBeitaSign=1: [-pi/2,pi/2]      
%               CosBeitaSign=-1:[ -pi,-pi/2 ] or [ pi/2,pi ]

function euler = C2Euler( C,rotateOrder,rotateDirection,CosBeitaSign )%#codegen



CheckError( C ) ; 
Nframes = size(C,3);
    

if numel(CosBeitaSign)==1
   CosBeitaSign = repmat( CosBeitaSign,Nframes,1 ); 
end

euler = zeros( 3,Nframes );
for k=1:Nframes
    C_k = C(:,:,k);
    euler(:,k) = C2Euler_One( C_k,rotateOrder,rotateDirection,CosBeitaSign(k) );
end
euler = MakeEuler_In2Pi( euler ) ;

% C:[3*3]
% 第二个欧拉角定义范围 CosBeitaSign = 1： [ -pi/2,pi/2 ]
%                    CosBeitaSign = -1： [ -pi,-pi/2 ] [ pi/2,pi ]
function eulerOut = C2Euler_One( C,rotateOrder,rotateDirection,CosBeitaSign )
coder.extrinsic('format');
format long

if isempty(rotateOrder)
    rotateOrder = 'ZYX';
end
if  isempty(rotateDirection)
    rotateDirection = [1,1,1];
end
if numel(rotateDirection)==1
   rotateDirection = repmat(rotateDirection,1,3); 
end
if  isempty(CosBeitaSign)
    CosBeitaSign = 1 ;
end

%% calculate euler angles of " CosBeitaSign=1 "

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
eulerOut = euler ;

% 第二个欧拉角定义范围 CosBeitaSign = 1： [ -pi/2,pi/2 ]
%                    CosBeitaSign = -1： [ -pi,-pi/2 ] [ pi/2,pi ]
if CosBeitaSign == -1
    eulerOut(1) = pi+euler(1) ;
    eulerOut(2) = pi-euler(2) ;
    eulerOut(3) = pi+euler(3) ;
    return;
end

function CheckError( C )
if size(C,1)~=3 || size(C,2)~=3
   errordlg('C2Euler:size(C) error'); 
end