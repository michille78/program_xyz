%% xyz  2015.4.9

% quaternion to Euler angle
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Input
% Q: [ NFrames*4 ]
% rotateOrder:  XYZ  XZY  ...
% rotateDirection: anticlockwise(1) or clockwise(-1)  [1,1,1]
% CosBeitaSign: 1 / -1 . 
%       CosBeitaSign = 1 : cos(beita)>0, beita [ -pi/2,pi/2 ]
%       CosBeitaSign = -1 : cos(beita)<0, beita [ -pi,-pi/2 ] or [ pi/2,pi ]
%%%  NOTE: There are two kinds of Euler Angles applied to the C. 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Output
% euler:  [N*3];   euler(k,:) = [eu_1,eu_2,eu_3];   rad   
%       store order of euler angle: the rotate order
%       eu_1,eu_3: [-pi,pi]
%       eu_2:   CosBeitaSign=1: [-pi/2,pi/2]      
%               CosBeitaSign=-1:[ -pi,-pi/2 ] or [ pi/2,pi ]

function euler = Q2Euler(Q,rotateOrder,rotateDirection,CosBeitaSign)

if ~exist('rotateOrder','var')
    rotateOrder = [];
end
if ~exist('rotateDirection','var')
    rotateDirection = [];
end
if ~exist( 'CosBeitaSign','var' )
    CosBeitaSign = [] ;    
end

C = Q2C(Q) ;
euler = C2Euler( C,rotateOrder,rotateDirection,CosBeitaSign ) ;
euler = MakeEuler_In2Pi( euler ) ;