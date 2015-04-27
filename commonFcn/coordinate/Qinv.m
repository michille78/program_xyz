%% xyz 2015.4.23

function invQ = Qinv( Q )
Q = Make_Const_N(Q,4);
N = size(Q,2);
invQ=  zeros(4,N);
for k=1:N
    invQ(:,k) = Qinv_One( Q(:,k) ) ;
end

function invQ = Qinv_One( Q )

invQ = Q;
invQ(2) = -invQ(2) ;
invQ(3) = -invQ(3) ;
invQ(4) = -invQ(4) ;
