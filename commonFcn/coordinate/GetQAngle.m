%% xyz 2015.4.23
% 

function angle = GetQAngle( Q )
Q = Make_Const_N(Q,4);
N = size(Q,2);
angle =  zeros(1,N);
for k=1:N
    angle(k) = GetQAngle_One( Q(:,k) ) ;
end


function angle = GetQAngle_One( Q )
cosHalfAngle = Q(1) ;
if sign(Q(2))~=0
    signQv = sign(Q(2)) ;   
else
    signQv = sign(Q(3))   
end
sinHalfAngle = normest(Q(2:4)) * signQv ;      % ºÜ´Ö²ÚµÄ ·ûºÅ ÅÐ¶Ï
angle = atan2( sinHalfAngle,cosHalfAngle );

