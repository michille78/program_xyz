%% 拆分 惯性和视觉数据


%% otherMakers
% otherMakers(k).MarkerSet
% otherMakers(k).frequency
% otherMakers(k).Position
% otherMakers(k).otherMakersN
%　otherMakers(k).time


%% InertialData
% InertialData.frequency (k)
% InertialData.time (k)
% InertialData.HipQuaternion(k)
% InertialData.HipPosition (k)
% InertialData.HeadQuaternion (k)
% InertialData.HeadPosition (k)
% InertialData.BodyDirection(k)
% InertialData.DataStyle(k)
% InertialData.HeadHipLength(k)


function SplitData()


dataFolder = 'E:\data_xyz\Hybrid Motion Capture Data\Head-2015-5.25\Head3';
InertialData = importdata( [dataFolder,'\InertialData.mat'] );
otherMakers = importdata( [dataFolder,'\otherMakers.mat'] );

global otherMakersTime  inertialTime 
global visionFre  inertialFre


visionFre  = otherMakers(1).frequency ;
[ otherMakersTime,otherMakersN ] = Get_otherMakersData( otherMakers );
inertialTime = InertialData.time ;
inertialFre = InertialData.frequency ;

vsN = length( otherMakers );
inerN = length(inertialTime);

T = min(inertialTime(inerN),otherMakersTime(vsN))  ;

splitT = (0:40)/40*T ;

splitInerN  = fix( splitT*inertialFre) ;
 splitInerN(1) = splitInerN(1)+1 ;
splitM = length(splitInerN);
splitVisionN = zeros(1,splitM);
TimeError = zeros(1,splitM);
for k=1:splitM
    [ splitVisionN(k),TimeError(k) ] = InertialK_to_VisionK( splitInerN(k) );
end


otherMakersCell = cell(1,splitM-1);
InertialDataCell = cell(1,splitM-1);
for k=1:splitM-1
    %% otherMakersCell
    otherMakers_k=struct;
    for i = splitVisionN(k):(splitVisionN(k+1)-1)
        j = i-splitVisionN(k)+1;
        if j==1
            otherMakers_k(j).frequency = visionFre ;
            otherMakers_k(j).MarkerSet = otherMakers(1).MarkerSet ;
        end
        otherMakers_k(j).Position = otherMakers(i).Position ;
        otherMakers_k(j).otherMakersN = otherMakers(i).otherMakersN ;
        otherMakers_k(j).time = otherMakers(i).time ;
    end
    save(  sprintf('%s\\otherMakers_%d.mat',dataFolder,k) ,'otherMakers_k' );
    otherMakersCell{k} = otherMakers_k ;
    %% InertialDataCell
    InertialData_k = struct;
    splitInerNum_k =splitInerN(k+1) - splitInerN(k) ;
    InertialData_k.frequency = inertialFre ;
    InertialData_k.HeadHipLength  = InertialData.HeadHipLength ;
    InertialData_k.BodyDirection  = InertialData.BodyDirection ;
    InertialData_k.DataStyle  = InertialData.DataStyle ;
    
    InertialData_k.time = InertialData.time( splitInerN(k):splitInerN(k+1)-1 );
    InertialData_k.HipPosition = InertialData.HipPosition( :,splitInerN(k):splitInerN(k+1)-1 );
    InertialData_k.HipQuaternion = InertialData.HipQuaternion( :,splitInerN(k):splitInerN(k+1)-1 );
    InertialData_k.HeadQuaternion = InertialData.HeadQuaternion( :,splitInerN(k):splitInerN(k+1)-1 );
    InertialData_k.HeadPosition = InertialData.HeadPosition( :,splitInerN(k):splitInerN(k+1)-1 );
    
    InertialDataCell{k} = InertialData_k ;
    save(  sprintf('%s\\InertialData_%d.mat',dataFolder,k) ,'InertialData_k' );
end

disp('split data OK')