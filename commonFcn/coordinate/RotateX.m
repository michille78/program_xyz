%% xyz 2015.3.25
%% anticlockwise = positive  rad
function Cx = RotateX( angX )
coder.inline('never');
Cx = [
    1       0           0
    0       cos(angX)   sin(angX)
    0       -sin(angX)  cos(angX)
];