%% xyz 

function RightHandMiddle1_new = Handle_RightHandMiddle1( RightHandMiddle1,BVHRotateRoder )

Nframes = size( RightHandMiddle1,1 );
RightHandMiddle1_new = zeros( Nframes,3 );
for k=1:Nframes
    RightHandMiddle1_new(k,:) = Handle_RightHandMiddle1_One( RightHandMiddle1(k,:),BVHRotateRoder ) ;
end

function RightHandMiddle1_new = Handle_RightHandMiddle1_One( RightHandMiddle1,BVHRotateRoder )

RightHandMiddle1 = RightHandMiddle1*pi/180 ;

RightHandMiddle1_ZYX = ChangeEulerRotate( RightHandMiddle1,BVHRotateRoder,'YXZ',[1 1 1],[1 1 1] ) ;
if RightHandMiddle1_ZYX(1)>0
    RightHandMiddle1_new = RightHandMiddle1 ;
else
    RightHandMiddle1_new = ChangeEulerBeitaScope( RightHandMiddle1 ) ;
end

RightHandMiddle1_new = RightHandMiddle1_new*180/pi ;
