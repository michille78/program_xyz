
clc
clear close all


BoneEnd0 = [ 0.034   -0.012   1.044 ];
BoneEnd1 = [ 0.002  -0.118 0.901 ];
BoneEnd2 = [ 0.135  0.070  0.919 ];

BoneEnd01 = BoneEnd0-BoneEnd1 ;
BoneEnd01_Norm = normest(BoneEnd01);

BoneEnd02 = BoneEnd0-BoneEnd2 ;
BoneEnd02_Norm = normest(BoneEnd02);

BoneEnd12 = BoneEnd0-BoneEnd2 ;
BoneEnd12_Norm = normest(BoneEnd12);


figure
plot3( BoneEnd0(1),BoneEnd0(2),BoneEnd0(3),'r.'  )
hold on
plot3( BoneEnd1(1),BoneEnd1(2),BoneEnd1(3),'r.'  )
plot3( BoneEnd2(1),BoneEnd2(2),BoneEnd2(3),'r.'  )

line( [ BoneEnd0(1) BoneEnd1(1) ],[ BoneEnd0(2) BoneEnd1(2) ],[ BoneEnd0(3) BoneEnd1(3) ] )
line( [ BoneEnd0(1) BoneEnd2(1) ],[ BoneEnd0(2) BoneEnd2(2) ],[ BoneEnd0(3) BoneEnd2(3) ] )
line( [ BoneEnd1(1) BoneEnd2(1) ],[ BoneEnd1(2) BoneEnd2(2) ],[ BoneEnd1(3) BoneEnd2(3) ] )

xlabel('x')
ylabel('y')
zlabel('z')

text( BoneEnd0(1),BoneEnd0(2),BoneEnd0(3),'BoneEnd0' )
text( BoneEnd1(1),BoneEnd1(2),BoneEnd1(3),'BoneEnd1' )
text( BoneEnd2(1),BoneEnd2(2),BoneEnd2(3),'BoneEnd2' )
