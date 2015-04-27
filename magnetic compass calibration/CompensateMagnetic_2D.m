%% xyz 2015 3.20
% Compensate the Magnetic (2D)

%%% Output: [M,b_n ]
% Hm_b: b_n frame H with errors
% H_b : b_n frame H without errors
% Hm_b = M*H_b+b_n   H_b = G_n*(Hm_b-b_n)   G_n=inv(M)

function H_n_xy_Rectified = CompensateMagnetic_2D( H_n_xy,G_n,b_n )

N = size(H_n_xy,1);
H_n_xy_Rectified = zeros(size(H_n_xy)) ;
for k=1:N
    H_n_xy_Rectified(k,:) = ( G_n(1:2,1:2)*( H_n_xy(k,1:2)'-b_n(1:2) ) )' ;
end
