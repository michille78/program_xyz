%% buaa xyz 2014.1.10 -> 2015.4.9

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Intput
% Q: quaternion from n to b
%       [ 4*Nframes ]
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Output
% C: [3*3*N]

function C = Q2C(Q) %#codegen

coder.inline('never');

Q = Make_Const_N( Q,4 ) ; 
Nframes = size( Q,2 );
C = zeros( 3,3,Nframes );
for k=1:Nframes
   C(:,:,k)  = Q2C_One( Q(:,k) );
end


function C = Q2C_One(Q)

coder.extrinsic('format');
coder.inline('never');

format long

Q=Q/norm(Q);
q0 = Q(1) ;
q1 = Q(2) ;
q2 = Q(3) ;
q3 = Q(4) ;
C=[   q0^2+q1^2-q2^2-q3^2,    2*(q1*q2+q0*q3),            2*(q1*q3-q0*q2);
        2*(q1*q2-q0*q3),        q0*q0-q1*q1+q2*q2-q3*q3,    2*(q2*q3+q0*q1);
        2*(q1*q3+q0*q2),        2*(q2*q3-q0*q1),            q0*q0-q1*q1-q2*q2+q3*q3     ];
