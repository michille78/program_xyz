%% xyz 2015.4.8
% Euler angle to direction cosine matrix

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Intput
% euler:  [3*N];   euler(k,:) = [eu_1,eu_2,eu_3];   rad   
%       store order of euler angle: the rotate order
%       eu_1,eu_3: [-pi,pi]
%       eu_2:   CosBeita=1: [-pi/2,pi/2]      
%               CosBeita=-1:[ -pi,-pi/2 ] or [ pi/2,pi ]
% rotateOrder:  XYZ  XZY  ...
% rotateDirection: anticlockwise(1) or clockwise(-1)  [1,1,1]
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Output
% C: [3*3*N]

function C =  Euler2C( euler,rotateOrder,rotateDirection )
coder.inline('never');
format long
if ~exist('rotateOrder','var')
    rotateOrder = [];
end
if ~exist('rotateDirection','var')
    rotateDirection = [];
end

euler = Make_Const_N( 3,euler ) ; 

Nframes = size( euler,2 );
C = zeros( 3,3,Nframes );
for k=1:Nframes
   C( :,:,k )  = Euler2C_One( euler(:,k),rotateOrder,rotateDirection );
end

function C =  Euler2C_One( euler,rotateOrder,rotateDirection )
coder.inline('never');

format long
if ~exist('rotateOrder','var') || isempty(rotateOrder)
    rotateOrder = 'ZYX';
end
if ~exist('rotateDirection','var') || isempty(rotateDirection)
    rotateDirection = [1,1,1];
end
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



