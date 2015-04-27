function C13 = CalculateC_Multi( C12,C23 )
if size(C12,3)==1
    C13 = C23*C12 ;
    return;
end

Nframes = size( C12,1 );
C13 = zeros(Nframes,3,3);
for k=1:Nframes
    C23_k = permute( C23(k,:,:),[2,3,1] );
    C12_k = permute( C12(k,:,:),[2,3,1] );
    C13(k,:,:) = C23_k*C12_k ;
end