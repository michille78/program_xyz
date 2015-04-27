%% xyz 2015.4.9

% NOTE : only euler2 need < CosBeita2 >

function euler2 = ChangeEulerRotate( euler1,rotateOrder1,rotateOrder2,rotateDirection1,rotateDirection2 )

if ~exist('rotateDirection1','var')
    rotateDirection1 = [1,1,1];
end
if ~exist('rotateDirection2','var')
    rotateDirection2 = [1,1,1];
end

CosBeitaSign1 = GetEulerCosBeitaSign( euler1 ) ;
CosBeitaSign2 = CosBeitaSign1 ;

C = Euler2C( euler1,rotateOrder1,rotateDirection1 );
euler2 = C2Euler( C,rotateOrder2,rotateDirection2,CosBeitaSign2 ) ;
