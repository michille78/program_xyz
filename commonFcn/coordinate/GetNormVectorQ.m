%% xyz 2015.4.28

%% Q = [ qs qv ] = [ qs qx qy qz ];  qs = cos(afa/2)
%% Qnew = [ qs_new qv_new ] = Q/ sin(afa/2) * sign(*) ;
%%% make:  
% 1) qs_new = cot(afa/2)     norm(qv_new)=1
% 2) qx_new>0
% or if abs(qx)<1e-3   qz_new>0
%% NOTE
% % when rotate angle is too small , calcualtion error of Qrb_NormVector is
% very big!
function Q_NormVector = GetNormVectorQ( Q )

coder.inline('never');

Q = Make_Const_N( Q,4 ) ; 
Nframes = size( Q,2 );
Q_NormVector = zeros(4,Nframes);
for k=1:Nframes
   Q_NormVector(:,k)  = GetNormVectorQ_One( Q(:,k) );
end

function Qnew = GetNormVectorQ_One( Q )
coder.inline('never');

Qv = [ Q(2) Q(3) Q(4) ];
Qnew = Q/normest(Qv) ;
%%% 2) make:   vector = Qnew(2:4,:)
% 1)   norm(vector(:,i))=1
% 2) if abs(vector(1))>1e-3  vector(1)>0
% elseif abs(vector(3))>1e-3   vector(3)>0
% elseif abs(vector(2))>1e-3   vector(2)>0

Qnew(2:4,:) = MakeVectorDirectionSame( Qnew(2:4,:) ) ;

