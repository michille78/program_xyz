%% xyz  2015.4.27
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Input
% Qnb
% Qwr
% Ypr  ：提前算好的转轴

%% 纯加计解算转动角度
function RotateAngle = CalculateRotateAngle_Acc( Qnb,Qwr,Ypr ) 

Nframes = length( Qnb ) ;
RotateAngle = zeros( 1,Nframes );
for k=1:Nframes
    A = CalculateA_One( Qnb(:,k),Qwr )  ;
    temp = -A(2,2:4)*Ypr / A(2,1) ;
    RotateAngle(k) = acot( temp )*2;
end
% figure
% plot(RotateAngle*180/pi)
% ylabel('RotateAngle');


function A = CalculateA_One( Qnb,Qwr ) 
if length(Qnb)==4
    Qbn = [ Qnb(1);-Qnb(2:4) ] ;
    LQMwr = LeftQMultiplyMatrix( Qwr ) ;
    RQMbn = RightQMultiplyMatrix( Qbn ) ;
    A = RQMbn * LQMwr ;
else
    A = NaN;
end

       