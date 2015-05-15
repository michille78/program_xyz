%% xyz 2015.5.13

%% 通过静止数据时，纯加计方法求得的转角方差，评价转轴计算精度
% AccZero_Num: 静止时的 序号

function YprAnalyzeStr = AnalyzeYpr( Qnb,Qwr,Ypr,AccZero_Num,YprName )

RotateAngle = CalculateRotateAngle_Acc( Qnb,Qwr,Ypr,AccZero_Num ) ;
std_RotateAngle = std(RotateAngle);
YprAnalyzeStr = sprintf( 'Ypr(%s)静止时转角标准差：%0.3f °',YprName,std_RotateAngle*180/pi );
