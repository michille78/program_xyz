

N = size(H_b,1);
H_b_Norm = zeros(1,N);
for k=1:N
    H_b_Norm(k) = normest( H_b(k,:) );
    if H_b_Norm(k)  < 10000
        k
    end
end
figure
plot( H_b_Norm )
disp('ok')