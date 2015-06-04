%% xyz 2015.3.15

% solving equation: Ax=b

function x = linearEquation( A,b )

format long  % must be format long

% [A_new,b_new ] = CondBalance( A,b ) ;
    
x = A\b ;

%% analyze the error of  solution

b_err = (A*x - b)./b ;  % the relative error
b_err_std = std(b_err);
b_err_mean = mean(b_err);
if abs(b_err_mean)>0.01
   errordlg(sprintf('the relative error mean of linear equation is too big! = %0.3f ' ,b_err_mean)); 
end
if abs(b_err_std)>0.05
   errordlg(sprintf('the relative error std of linear equation is too big! = %0.3f ' ,b_err_std)); 
end

fprintf( ' linear equation solving relative error\n \t error mean = %0.3f,error std = %0.3f \n ',b_err_mean,b_err_std );

figure( 'name','LinearEquation Error' )
plot( b_err )


function [A_new,b_new ] = CondBalance( A,b )


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

