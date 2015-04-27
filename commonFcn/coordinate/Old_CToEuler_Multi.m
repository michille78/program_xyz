%% xyz  2015.4.2

% direction cosine matrix  to Euler angle  for multi C
% C: [N*3*3]
%       euler = [eu_x,eu_y,eu_z]  ( Nframes*3 )
% store order of euler angle: the rotate order
% positive:anticlockwise rotate 
% the first and third euler angles scope: [-pi,pi]
% the second euler angles scope: [-pi/2,pi/2]
% unit:rad 
% rotateDirection: anticlockwise(1) or clockwise(-1)  111

function euler = CToEuler_Multi( C,rotateOrder,rotateDirection )
format long

if ~exist('rotateOrder','var')
    rotateOrder = 'ZYX';
end
if ~exist('rotateDirection','var')
    rotateDirection = [1,1,1];
end

Nframes = size( C,1 );
euler = zeros( Nframes,3 );
for k=1:Nframes
    C_K = permute( C(k,:,:),[2,3,1] );
    euler(k,:) = CToEuler( C_K,rotateOrder,rotateDirection );
end
