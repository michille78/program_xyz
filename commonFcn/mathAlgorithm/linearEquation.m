%% xyz 2015.3.15

% solving equation: Ax=b

function x = linearEquation( A,b )

format long  % must be format long

condA = cond(A) ;
% balance method for sick of equations
N = size(A,1);
D = zeros(N,N) ;
for k=1:N
    [ maxA_k,i ] = max( abs(A(k,:)) ) ;
    maxA_k = maxA_k*sign(A(k,i)) ;
    D(k,k) = 1/maxA_k ; 
end

A_new = D*A ;
b_new = D*b ;

condA_New = cond(A) ;
fprintf('linear equation:\n cond A before balance = %f \n cond after balance = %f\n',condA,condA_New  ) ;
% least square linear equation
if condA_New<condA
   A = A_new ;
   b = b_new ;
end
    
x = A\b ;

