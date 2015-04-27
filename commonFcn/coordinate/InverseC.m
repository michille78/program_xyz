%% xyz 2015.4.8

% C12: [N*3*3]

function C21 = InverseC( C12 )
coder.inline('never');

Nframes = size( C12,1 );
C21 = zeros(3,3,Nframes);
for k=1:Nframes
    C21(:,:,k) = C12(:,:,k)' ;
end