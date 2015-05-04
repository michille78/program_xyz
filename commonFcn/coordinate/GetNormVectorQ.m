%% xyz 2015.4.28

%% get newQ that  normest of qv is 1 and direction the same 

% Q = [ qs qv ] = [ qs qx qy qz ];  qs = cos(afa/2)
% Qnew = [ qs_new qv_new ] = Q/ sin(afa/2) * sign(*) ;

%% make direction of qv  the same
% Q_new = Q * K   [4*N]
% Q  = [  qs qv  ];
%%% make:  
% 1)   norm(qv(:,i))=1
% 2) if abs(qv(1))>1e-3  qv(1)>0
% elseif abs(qv(3))>1e-3   qv(3)>0
% elseif abs(qv(2))>1e-3   qv(2)>0
%% NOTE
% % when rotate angle is too small , calcualtion error of Qrb_NormVector is
% very big!

function [ Q_NormVector,sinHalfAfaSign ] = GetNormVectorQ( Q )

format long
coder.inline('never');

Q = Make_Const_N( Q,4 ) ; 
Nframes = size( Q,2 );
sinHalfAfaSign = zeros(1,Nframes);  % sign of sinHalfAfa
N = size(Q,2);
Q_NormVector = zeros(4,N);
for k=1:Nframes
   [ Q_NormVector(:,k),sinHalfAfaSign(k) ]  = GetNormVectorQ_One( Q(:,k) );
end

function [ Q,sinHalfAfaSign ] = GetNormVectorQ_One( Q )

%% make norm(qv)=1
qv = [ Q(2);Q(3);Q(4) ];
Q = Q/normest( qv ) ;
%% make direction of qv  the same
[ ~,sinHalfAfaSign ] = MakeVectorDirectionSame( qv ) ;
Q = Q*sinHalfAfaSign ;

