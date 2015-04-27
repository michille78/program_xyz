%% xyz 2015.4.9
% Euler angle to quaternion

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Intput
% euler:  [N*3];   euler(k,:) = [eu_1,eu_2,eu_3];   rad   
%       store order of euler angle: the rotate order
%       eu_1,eu_3: [-pi,pi]
%       eu_2:   CosBeita=1: [-pi/2,pi/2]      
%               CosBeita=-1:[ -pi,-pi/2 ] or [ pi/2,pi ]
% rotateOrder:  XYZ  XZY  ...
% rotateDirection: anticlockwise(1) or clockwise(-1)  [1,1,1]
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Output
% Q: [N*4]

function Q = Euler2Q( euler,rotateOrder,rotateDirection )

coder.inline('never');
format long

euler = Make_Const_N( euler,3 ) ; 
Nframes = size( euler,2 );
Q = zeros( 4,Nframes );
for k=1:Nframes
   Q(:,k)  = Euler2Q_One( euler(:,k),rotateOrder,rotateDirection );
end

function Q = Euler2Q_One( euler,rotateOrder,rotateDirection )
format long

if ~exist('rotateOrder','var')
    rotateOrder = 'ZYX';
end
if ~exist('rotateDirection','var')
    rotateDirection = [1,1,1];
end
for i=1:3
    euler(i) = euler(i)*rotateDirection(i) ;
end

switch rotateOrder
    case 'ZYX' 
        Q = MultiplyThreeQ( QZ(euler(1)),QY(euler(2)),QX(euler(3)) ) ;
    case 'ZXY'
        Q = MultiplyThreeQ( QZ(euler(1)),QX(euler(2)),QY(euler(3)) ) ;
    case 'XYZ'
        Q = MultiplyThreeQ( QX(euler(1)),QY(euler(2)),QZ(euler(3)) ) ;
    case 'XZY'
        Q = MultiplyThreeQ( QX(euler(1)),QZ(euler(2)),QY(euler(3)) ) ;
    case 'YXZ'
        Q = MultiplyThreeQ( QY(euler(1)),QX(euler(2)),QZ(euler(3)) ) ;
    case 'YZX'
        Q = MultiplyThreeQ( QY(euler(1)),QZ(euler(2)),QX(euler(3)) ) ;
end
Q = Q/norm(Q) ;

%% Euler X to Q
function QX = QX( eulerX )
QX = [ cos(eulerX/2)  sin(eulerX/2) 0 0 ];

%% Euler Y to Q
function QY = QY( eulerY )
QY = [ cos(eulerY/2)  0  sin(eulerY/2) 0 ];

%% Euler Z to Q
function QZ = QZ( eulerZ )
QZ = [ cos(eulerZ/2) 0 0 sin(eulerZ/2) ];

%% Q = Q1oQ2oQ3
function Q = MultiplyThreeQ( Q1,Q2,Q3 )
Q12 = QuaternionMultiply(Q1,Q2) ;
Q = QuaternionMultiply(Q12,Q3) ;

