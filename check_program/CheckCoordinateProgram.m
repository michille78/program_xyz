%% xyz 2015.3.6
% 坐标系转换函数校验：1）判断欧拉角的转动顺序和存储顺序

function CheckCoordinateProgram()

Q = [ -0.3203	0.2815	0.5924	-0.6836  ];
Euler = [ -133.21 -9.93 0.50 ]  % Y X Z

 Q = [ 0.888278782 0 0.459304661 0 ] ;
% Q = [ 0.999781728 0.0208942499 0 0];


Euler_ZYX = FQtoEuler(Q,'ZYX')*180/pi
Euler_ZYX_NT = FQtoEuler_NT(Q,'ZYX')

Euler_YZX = FQtoEuler(Q,'YZX')*180/pi
Euler_YZX_NT = FQtoEuler_NT(Q,'YZX')

Euler_YZX = FQtoEuler(Q,'ZXY')*180/pi
Euler_YZX_NT = FQtoEuler_NT(Q,'ZXY')

Euler_YXZ = FQtoEuler(Q,'YXZ')*180/pi
Euler_YXZ_NT = FQtoEuler_NT(Q,'YXZ')

Euler_XYZ = FQtoEuler(Q,'XYZ')*180/pi
Euler_XYZ_NT = FQtoEuler_NT(Q,'XYZ')

Euler_XZY = FQtoEuler(Q,'XZY')*180/pi
Euler_XZY_NT = FQtoEuler_NT(Q,'XZY')


%% 我纠正的顺序
Euler_XYZ = FQtoEuler(Q,'XYZ')*180/pi
Euler_YXZ_NT = FQtoEuler_NT(Q,'YXZ')

Euler_YZX = FQtoEuler(Q,'YZX')*180/pi
Euler_XZY_NT = FQtoEuler_NT(Q,'XZY')



disp('OK')





