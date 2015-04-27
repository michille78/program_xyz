%% Change Euler Rotate Order

function euler2 = ChangeEulerRotate_c( euler1,rotateOrder1,rotateOrder2,rotateDirection1,rotateDirection2 )
coder.inline('never');
C = EulerToC_c( euler1,rotateOrder1,rotateDirection1 );
euler2 = CToEuler_c( C,rotateOrder2,rotateDirection2 ) ;
