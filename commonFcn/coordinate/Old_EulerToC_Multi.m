%% xyz 2015.4.2
% Euler angle to direction cosine matrix  for N*3 eluer

%       euler(k,:) = [eu_x,eu_y,eu_z] 
% store order of euler angle: the rotate order
% positive:anticlockwise rotate 
% euler angle scope: [-pi,pi]
% unit:rad 
% rotateDirection: anticlockwise(1) or clockwise(-1)  111


function C =  EulerToC_Multi( euler,rotateOrder,rotateDirection )

format long
if ~exist('rotateOrder','var')
    rotateOrder = 'ZYX';
end
if ~exist('rotateDirection','var')
    rotateDirection = [1,1,1];
end

if size(euler,2)~=3
    C = NaN ;
   errordlg('EulerToC_Multi: size of euler Wrong'); 
   return;
end
Nframes = size( euler,1 );
C = zeros( Nframes,3,3 );
for k=1:Nframes
   C( k,:,: )  = EulerToC( euler(k,:),rotateOrder,rotateDirection );
end
