%% xyz 2015.4.27


%% Calculate Rotate Vector only by Acc 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Input
% Qnb_ZeroCal�� ת�����ʱ��ε� Qnb
% Qwr�� ��ʼʱ�̵ĵ���ϵ �� ��ʼʱ�̵ı���ϵ
% AHRSThreshod�� ����ж�ָ��
% SDOFStaticFlag�� Qnb_ZeroCal���Ƿ�0���ٶ��жϽ��

function [ Ypr,RecordStr ] = GetRotateVector_Acc( Qnb_ZeroCal,Qwr,AHRSThreshod,SDOFStaticFlag )
RoateVectorCalMinAngleFirst = AHRSThreshod.RoateVectorCalMinAngleFirst ;
RoateVectorCalMinAngleSecond = AHRSThreshod.RoateVectorCalMinAngleSecond ;
RoateVectorCalMinAngleScope = AHRSThreshod.RoateVectorCalMinAngleScope ;
RoateVectorCalMinAngleScopeSub = AHRSThreshod.RoateVectorCalMinAngleScopeSub ;
%%% ����ת�᣺ ���躽��Ϊ0������ѡ������ת���������ݣ�ת�ǽϴ�
[ Qnb_RCD,Qwr_RCD,RecordStr1 ] = SelectRotateVectorCalcualteData_First( Qnb_ZeroCal,Qwr,RoateVectorCalMinAngleFirst,SDOFStaticFlag,RoateVectorCalMinAngleScope,RoateVectorCalMinAngleScopeSub ) ;
dbstop in CalculateRotateVector_Acc
Ypr1 = CalculateRotateVector_Acc( Qnb_RCD,Qwr_RCD ) ;
%%% ����ת�᣺ 
[ Qnb_RCD,Qwr_RCD,RecordStr2 ] = SelectRotateVectorCalcualteData_Second...
    ( Qnb_ZeroCal,Qwr,Ypr1,RoateVectorCalMinAngleSecond,RoateVectorCalMinAngleScope,RoateVectorCalMinAngleScopeSub,SDOFStaticFlag ) ;
Ypr = CalculateRotateVector_Acc( Qnb_RCD,Qwr_RCD ) ;

Ypr1Str = sprintf( '%0.4f  ',Ypr1 );
Ypr2Str = sprintf( '%0.4f  ',Ypr );
RecordStr = sprintf( '%s Ypr1 = %s  \n %s Ypr2 = %s \n',RecordStr1,Ypr1Str,RecordStr2,Ypr2Str );
disp( RecordStr );

%% �����ʺ�ת�Ǽ������Ԫ�� Qnb_RCD �� Qwr_RCD ����ת�� Ypr
function  [ Ypr,RotateAngle_RCD ] = CalculateRotateVector_Acc( Qnb_RCD,Qwr_RCD )
D = CalculateD( Qnb_RCD,Qwr_RCD ) ;

DTD = D'*D ;
[ eigV,eigD ] = eig( DTD );
eigValue = diag(eigD);
[ minEigValue,minEig_k ] = min( eigValue );
X = eigV( :,minEig_k );
X = X/normest(X( 1:3 )) ;
Ypr = X( 1:3 );

RotateAngle_RCD= (acot(X(4:length(X))))*180/pi*2 ;


%% check
K = size(D,2)-3;
As = zeros( 2,K );
for i=1:K
    As(:,i) = D( i*2-1:i*2,3+i );
end

DX = D*X ;
DTDX = DTD*eigV( :,1 ) ;

function D = CalculateD( Qnb,Qwr )
Nframes = size(Qnb,2);
D = zeros( 2*Nframes,3+Nframes );
for i=1:Nframes
    Ai = CalculateA_One( Qnb(:,i),Qwr ) ;
    As_i = Ai(2:3,1);
    Av_i = Ai(2:3,2:4);
    D( 2*i-1:2*i,1:3 ) = Av_i ;
    D( 2*i-1:2*i,3+i ) = As_i ;
end
disp('')

function A = CalculateA_One( Qnb,Qwr ) 
if length(Qnb)==4
    Qbn = [ Qnb(1);-Qnb(2:4) ] ;
    LQMwr = LeftQMultiplyMatrix( Qwr ) ;
    RQMbn = RightQMultiplyMatrix( Qbn ) ;
    A = RQMbn * LQMwr ;
else
    A = NaN;
end

%% Second : select data be suitable for rotate vector calculating
%%% ���ݵ�һ�μ���Ĵ���ת�� Ypr������ת���Ƕȣ� ���� RoateVectorCalMinAngleSecond
%%% ��������Ϊ����Ч�ģ����ڽ��еڶ���ת�����
function [ Qnb_RCD,Qwr_RCD,RecordStr ] = SelectRotateVectorCalcualteData_Second...
    ( Qnb_ZeroCal,Qwr,Ypr,RoateVectorCalMinAngleSecond,RoateVectorCalMinAngleScope,RoateVectorCalMinAngleScopeSub,SDOFStaticFlag )
RotateAngleSecond = CalculateRotateAngle_Acc( Qnb_ZeroCal,Qwr,Ypr ) ;
%% �ڶ�������ѡ�����
% 1�� ת�Ǵ��� RoateVectorCalMinAngleSecond
% 2�� ��ֹ״̬
% 3�� �Ƕȷ�Χ���� 
IsAngleBig = abs(RotateAngleSecond)>RoateVectorCalMinAngleSecond ;
IsAngleBigStatic1 = IsAngleBig & SDOFStaticFlag.IsSDOFAccelerationZero(1:length(IsAngleBig))' ;
IsAngleBigStatic2 = IsAngleBig & SDOFStaticFlag.IsSDOFAccelerationToHeartZero(1:length(IsAngleBig))' ;
IsAngleBigStatic3 = IsAngleBig & SDOFStaticFlag.IsAccNormZero(1:length(IsAngleBig))' ;

%% ����ѡ���ϸ�� 0 ���ٶ��ж�

IsAngleBigStatic_SeclectFlag = 0 ;
if ~isempty(IsAngleBigStatic1>0)
    [ AngleScope1,AngleScopeSub1 ] = GetAngleScope( RotateAngleSecond(IsAngleBigStatic1) ) ;
    if AngleScope1 > RoateVectorCalMinAngleScope
        if AngleScopeSub1 > RoateVectorCalMinAngleScopeSub
            %%% ѡ���ϸ��0���ٶȱ�׼ʱ��������
            IsAngleBigStatic = IsAngleBigStatic1 ;
            IsAngleBigStatic_SeclectFlag = 1 ;
        end
    end
end
%% ���ѡ�����ļ��ٺ�ģ���ж�����  IsSDOFAccelerationToHeartZero
if IsAngleBigStatic_SeclectFlag == 0
    if ~isempty( IsAngleBigStatic2>0 )
        [ AngleScope2,AngleScopeSub2 ] = GetAngleScope( RotateAngleSecond(IsAngleBigStatic2) ) ;
        if AngleScope2 > RoateVectorCalMinAngleScope
            if AngleScopeSub2 > RoateVectorCalMinAngleScopeSub
                IsAngleBigStatic = IsAngleBigStatic2 ;
                IsAngleBigStatic_SeclectFlag = 2 ;
            end
        end
    end
end
%% �����ѡ��ģ���ж�����  IsAccNormZero
if IsAngleBigStatic_SeclectFlag == 0
    if ~isempty( IsAngleBigStatic3>0 )
        [ AngleScope3,AngleScopeSub3 ] = GetAngleScope( RotateAngleSecond(IsAngleBigStatic3) ) ;
        if AngleScope3 > RoateVectorCalMinAngleScope
            if AngleScopeSub3 > RoateVectorCalMinAngleScopeSub
                IsAngleBigStatic = IsAngleBigStatic3 ;
                IsAngleBigStatic_SeclectFlag = 3 ;
            end
        end
    end
end
RecordStr = sprintf( 'SelectRotateVectorCalcualteData_Second ת������ѡ���־ IsAngleBigStatic_SeclectFlag = %0.0f \n',IsAngleBigStatic_SeclectFlag );
%% ���Ͼ�������ʱ˵���Ҳ�����ת�������
if IsAngleBigStatic_SeclectFlag == 0
   errordlg( '�Ҳ���ת��������ݣ�SelectRotateVectorCalcualteData_First��' ); 
   Qnb_RCD = [];
   Qwr_RCD = [];
   return;
end

Qnb_RCD = Qnb_ZeroCal( :,IsAngleBigStatic );
Qwr_RCD = Qwr;

%% check
RotateAngleSecond_RCD = RotateAngleSecond(IsAngleBigStatic);
time = 1:length(RotateAngleSecond);
figure
plot( RotateAngleSecond*180/pi )
hold on
plot( time(IsAngleBigStatic),RotateAngleSecond_RCD*180/pi,'r.' )

function [ AngleScope,AngleScopeSub ] = GetAngleScope( RotateAngleSeclected )
AngleScope = max( RotateAngleSeclected ) - min( RotateAngleSeclected ) ;
% ��ֵ�ĸ��Ƿ�Χ
RotateAngleSeclectedPos = RotateAngleSeclected(RotateAngleSeclected>0) ;
if ~isempty( RotateAngleSeclectedPos  )
    AngleScopePositive = max( RotateAngleSeclectedPos ) - min( RotateAngleSeclectedPos ) ;
else
    AngleScopePositive = 0 ;
end
% ��ֵ�ĸ��Ƿ�Χ
RotateAngleSeclectedNeg = RotateAngleSeclected(RotateAngleSeclected<0) ;
if ~isempty( RotateAngleSeclectedNeg  )
    AngleScopeNegtive = min( RotateAngleSeclectedNeg ) - max( RotateAngleSeclectedNeg ) ;
else
    AngleScopeNegtive = 0;
end

AngleScopeSub = max( AngleScopePositive,AngleScopeNegtive );

%% Firts : select data be suitable for rotate vector calculating
%%% ���躽�򱣳�0ʱ�������ͺ��ת����Ԫ����ת�Ǵ��� RoateVectorCalMinAngleFirst �Ƕ�ʱ��������ת��ĵ�һ�μ���
function [ Qnb_RCD,Qwr_RCD,RecordStr ] = SelectRotateVectorCalcualteData_First( Qnb,Qwr,RoateVectorCalMinAngleFirst,SDOFStaticFlag,RoateVectorCalMinAngleScope,RoateVectorCalMinAngleScopeSub )

Qrw = Qinv( Qwr );
N = size(Qnb,2);
Qrb_false = QuaternionMultiply( repmat(Qrw,1,N),Qnb );
angleFirst = GetQAngle( Qrb_false ) ;

IsAngleBig =  angleFirst > RoateVectorCalMinAngleFirst | angleFirst < -RoateVectorCalMinAngleFirst ;

IsAngleBigStatic1 = IsAngleBig & SDOFStaticFlag.IsSDOFAccelerationZero(1:length(IsAngleBig))' ;
IsAngleBigStatic2 = IsAngleBig & SDOFStaticFlag.IsSDOFAccelerationToHeartZero(1:length(IsAngleBig))' ;
IsAngleBigStatic3 = IsAngleBig & SDOFStaticFlag.IsAccNormZero(1:length(IsAngleBig))' ;

%% ����ѡ���ϸ�� 0 ���ٶ��ж�

IsAngleBigStatic_SeclectFlag = 0 ;
if ~isempty(IsAngleBigStatic1>0)
    [ AngleScope1,AngleScopeSub1 ]  = GetAngleScope( angleFirst(IsAngleBigStatic1) ) ;
    if AngleScope1 > RoateVectorCalMinAngleScope 
        if AngleScopeSub1 > RoateVectorCalMinAngleScopeSub
            %%% ѡ���ϸ��0���ٶȱ�׼ʱ��������
            IsAngleBigStatic = IsAngleBigStatic1 ;
            IsAngleBigStatic_SeclectFlag = 1 ;
        end
    end
end
%% ���ѡ�����ļ��ٺ�ģ���ж�����  IsSDOFAccelerationToHeartZero
if IsAngleBigStatic_SeclectFlag == 0
    if ~isempty( IsAngleBigStatic2>0 )
        [ AngleScope2,AngleScopeSub2 ]  = GetAngleScope( angleFirst(IsAngleBigStatic2) ) ;
        if AngleScope2 > RoateVectorCalMinAngleScope
            if AngleScopeSub2 > RoateVectorCalMinAngleScopeSub
                IsAngleBigStatic = IsAngleBigStatic2 ;
                IsAngleBigStatic_SeclectFlag = 2 ;
            end
        end
    end
end
%% �����ѡ��ģ���ж�����  IsAccNormZero
if IsAngleBigStatic_SeclectFlag == 0
    if ~isempty( IsAngleBigStatic3>0 )
        [ AngleScope3,AngleScopeSub3 ]  = GetAngleScope( angleFirst(IsAngleBigStatic3) ) ;
        if AngleScope3 > RoateVectorCalMinAngleScope
            if AngleScopeSub3 > RoateVectorCalMinAngleScopeSub
                IsAngleBigStatic = IsAngleBigStatic3 ;
                IsAngleBigStatic_SeclectFlag = 3 ;
            end
        end
    end
end
RecordStr = sprintf( 'SelectRotateVectorCalcualteData_First ת������ѡ���־ IsAngleBigStatic_SeclectFlag = %0.0f \n',IsAngleBigStatic_SeclectFlag );
%% ���Ͼ�������ʱ˵���Ҳ�����ת�������
if IsAngleBigStatic_SeclectFlag == 0
   errordlg( '�Ҳ���ת��������ݣ�SelectRotateVectorCalcualteData_First��' ); 
   Qnb_RCD = [];
   Qwr_RCD = [];
   return;
end

Qnb_RCD = Qnb( :,IsAngleBigStatic );
Qwr_RCD = Qwr;

%% check
angleFirst_RCD = angleFirst(IsAngleBigStatic);
time = 1:N;
figure
plot( angleFirst*180/pi )
hold on
plot( time(IsAngleBigStatic),angleFirst_RCD*180/pi,'r.' )

