%% xyz 2015.3.25
% anticlockwise = positive


%% anticlockwise rotate is positive  rad
function Cy = RotateY( angY )
coder.inline('never');
Cy = [
    cos(angY)       0       -sin(angY)
    0               1       0
    sin(angY)       0       cos(angY)
];