function C21 = InverseC_Multi( C12 )
if size(C12,3)==1
    C21 = C12';
    return;
end

Nframes = size( C12,1 );
C21 = zeros(Nframes,3,3);
for k=1:Nframes
    C12_k = permute( C12(k,:,:),[2,3,1] );
    C21(k,:,:) = C12_k' ;
end