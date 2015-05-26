%% noitom xyz 2015 5 22

%% otherMakers
% otherMakers.frequency
% otherMakers.Position
% otherMakers.otherMakersN

%% InertialData
% InertialData.frequency 
% InertialData.time 
% InertialData.HipQuaternion
% InertialData.HipPosition 
% InertialData.HeadQuaternion 
% InertialData.HeadPosition 
% InertialData.BodyDirection
% InertialData.DataStyle
% InertialData.HeadHipLength

function ReadINSVNSData(  ) 

dataFolder = 'E:\data_xyz\Hybrid Motion Capture Data\Head\Head1';

if ~exist([dataFolder,'\otherMakers.mat'])
    VisionDataCell = importdata([dataFolder,'\opt.txt']);
    otherMakers = ReadOtherMakers( VisionDataCell ) ;
    
    %% the maker set segment
    otherMakers(1).MarkerSet = 'Head';
    
    save([dataFolder,'\otherMakers.mat'],'otherMakers')
else
    otherMakers = importdata([dataFolder,'\otherMakers.mat']);
end

if ~exist([dataFolder,'\InertialData.mat'])
    InitialDataCell = importdata( [dataFolder,'\inertial.txt'] );
    InertialData = ReadInertialData( InitialDataCell ) ;
    save([dataFolder,'\InertialData.mat'],'InertialData')
else
    InertialData = importdata([dataFolder,'\InertialData.mat']);
end

N = length(InertialData.time) ;
fprintf( 'initial time = %0.3f s,  N = %d \n',InertialData.time(N)/1000,N );
N = length(otherMakers);
fprintf( 'optitrack time = %0.3f s,  N = %d \n',otherMakers(N).time/1000,N);


DrawINSVNS( InertialData,otherMakers );

disp('Read INS-VNS Data OK')

%% Draw initial and vision data
function DrawINSVNS( InertialData,otherMakers )
% both data are defined in left-up-front frame
% xz plane position
HeadPosition = InertialData.HeadPosition;
figure('name','xy plane trace')
plot( HeadPosition(1,:),HeadPosition(3,:) );
hold on
plot( HeadPosition(1,:),HeadPosition(3,:) );

%% Read Initial Data
function InertialData = ReadInertialData( InertialDataCell )
% Body Direction
BodyDirectionCell = textscan( InertialDataCell{1},'Body Direction:%f %f %f' );
BodyDirection = [BodyDirectionCell{1};BodyDirectionCell{2};BodyDirectionCell{3}] ;
% DataStyle
InertialData.DataStyle = InertialDataCell{2} ;

% time
InertialDataCell(1:3) = [];
N = length(InertialDataCell);
time = zeros(1,N);
HipQuaternion = zeros(4,N);
HipPosition = zeros(3,N);
HeadQuaternion = zeros(4,N);
HeadPosition = zeros(3,N);
for k=1:N
    InertialDataCell_k = InertialDataCell{k};
    timeStr_k = textscan( InertialDataCell_k,'%s',1 );
    time(k) = TransformTimeFormat( timeStr_k{1} );
    
    m = FindSecondSpace( InertialDataCell_k,1 ) ;
    InertialDataCell_k(1:m) = [];
    C = textscan( InertialDataCell_k,'%f',7*2 );
    C = C{1};
    HipQuaternion(:,k) = [ C(1);C(2);C(3);C(4) ];
    HipPosition(:,k) = [ C(5);C(6);C(7) ];
    HeadQuaternion(:,k) = [ C(8);C(9);C(10);C(11) ];
    HeadPosition(:,k) = [ C(12);C(13);C(14) ];
end

time = HandleTimeScope( time ) ;
frequency = N/time(N) ;

InertialData.BodyDirection = BodyDirection;
InertialData.frequency = frequency ;
InertialData.time = time ;
InertialData.HipQuaternion = HipQuaternion ;
InertialData.HipPosition = HipPosition ;
InertialData.HeadQuaternion = HeadQuaternion ;
InertialData.HeadPosition = HeadPosition ;
InertialData.HeadHipLength = HipPosition(3,1)-HeadPosition(3,1) ;   % Hip与Head的高度差

%% read OtherMakers Data
function otherMakers = ReadOtherMakers( VisionDataCell )
VisionDataCell(1)=[];
N = length(VisionDataCell);
time = zeros( 1,N );
otherMakers = struct;

for k=1:N
   VisionDataCell_k =  VisionDataCell{k};
   C = textscan( VisionDataCell_k,'%s %d',1 );
   timeStr_k = C{1};
   otherMakersN = C{2};
   m = FindSecondSpace( VisionDataCell_k,2 );
   VisionDataCell_k_Temp = VisionDataCell_k( m+1:length(VisionDataCell_k) );
   otherMakers_Data_k = zeros(3,otherMakersN);
   for i=1:otherMakersN
        otherMakers_Data_k_Cell = textscan( VisionDataCell_k_Temp,'%f %f %f',1 );
        otherMakers_Data_k(:,i) = cell2mat(otherMakers_Data_k_Cell);
   end
   
   otherMakers(k).otherMakersN = otherMakersN ;
   otherMakers(k).Position = otherMakers_Data_k ;
   time(k) = TransformTimeFormat( timeStr_k );
end
timeNew = HandleTimeScope( time );
for k=1:N
    otherMakers(k).time = timeNew(k);
end

frequency = N/timeNew(N) ;

otherMakers(1).frequency = frequency ;
 

%% 得到字符号串 第spaceSeg_n段空格 的 最后一个空格序号
function m = FindSecondSpace( Str,spaceSeg_n )
spaceNum = strfind(Str,' ') ;

k = 2 ;
while k<=length(spaceNum)
   if  spaceNum(k) == spaceNum(k-1)+1
       spaceNum(k-1) = [];
   else
       k = k+1;
   end
end
m = spaceNum(spaceSeg_n);


%% 将时间转为毫秒形式
function time_s = TransformTimeFormat( timeStr )
format long
C = textscan( timeStr{1},'%f:%f:%f');
time_ms = C{1}*60*1000 + C{2}*1000 + C{3} ;
time_s = time_ms/1000 ;

function Time = HandleTimeScope( Time )
N = length(Time);
HourChange_k = 0;
for k=1:N
   if k>1
      if Time(k)<Time(k-1) 
          HourChange_k = k ;
          break;
      end
   end
end
if HourChange_k~=0
    for k=HourChange_k:N
       Time(k) = Time(k) + 1*60*1000 ; 
    end
end

time0 = Time(1);
for k=1:N
   Time(k) =  Time(k) - time0  ;  
end
