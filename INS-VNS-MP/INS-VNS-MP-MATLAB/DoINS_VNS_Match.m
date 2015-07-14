%% xyz  2015.6.30

function DoINS_VNS_Match(  )

clc
clear all
% close all
global dataFolder 
% dataFolder = 'E:\data_xyz\Hybrid Motion Capture Data\6.25\三点-综合';
 dataFolder = 'E:\data_xyz\Hybrid Motion Capture Data\7.2 dataB\T2';
% dataFolder =  'E:\data_xyz\Hybrid Motion Capture Data\7.6data\1Person3Points_2';
dataName1 = 'CalData0';
dataName2 = 'CalData1';
 dataName = 'CalculationData';

%% load INS data   1
BothCutT = 8;
INSCutT = BothCutT+4.01;
INS_StartK = fix( INSCutT*120);
VNSstartK = fix(BothCutT*120.0158);

fprintf('INS_StartK = %0.0f \n',INS_StartK);

[ HeadA1,LeftHandA1,RightHandA1 ] = ReadINSAccData( dataFolder,dataName1,INS_StartK,2 ) ;
[ HeadA2,LeftHandA2,RightHandA2 ] = ReadINSAccData( dataFolder,dataName2,INS_StartK,1 ) ;
[ HeadP1,LeftHandP1,RightHandP1 ] = ReadINSPData( dataFolder,dataName1,INS_StartK ) ;
[ HeadP2,LeftHandP2,RightHandP2 ] = ReadINSPData( dataFolder,dataName2,INS_StartK ) ;

inertialN = size(HeadA1,2);
INSA_All = [ HeadA1; LeftHandA1;RightHandA1;HeadA2; LeftHandA2;RightHandA2 ];
% INSA_All = [ HeadA1; LeftHandA1;RightHandA1 ];
 INSPosition = [ HeadP1;LeftHandP1;RightHandP1;HeadP2;LeftHandP2;RightHandP2 ];
% INSPosition = [ HeadP1;LeftHandP1;RightHandP1 ];
save([dataFolder,'\INSPosition.mat'],'INSPosition')

inertialFre = 120;

%% load INS data   2
% [ HeadA,LeftHandA,RightHandA ] = ReadINSAccData( dataFolder,dataName,1 ) ;
% [ HeadP,LeftHandP,RightHandP ] = ReadINSPData( dataFolder,dataName,1 ) ;
% inertialN = size(HeadA,2);
% INSA_All = [ HeadA; LeftHandA;RightHandA ];
% INSPosition = [ HeadP;LeftHandP;RightHandP];
% save([dataFolder,'\INSPosition.mat'],'INSPosition')
% inertialFre = 96;
%% load VNS data
otherMakers = ReadOptitrack( dataFolder,'\Opt.txt' );
otherMakers = CutData( otherMakers,VNSstartK );
otherMakers = FullotherMakersField( otherMakers );

visualFre = otherMakers(1).frequency;

%%
MinMatchDegree = 1;
% dbstop in INS_VNS_Match

wh = waitbar(0,'DoINS_VNS_Match');

vk_last = 0;
for j=1:inertialN
    INSA_All_k = INSA_All(:,j);
    
    otherMakersPosition_k = [];
    otherMakersN_k = [] ;

%     dbstop in INS_VNS_Match
    [ matchedDegree,INSA_TestOut,~,~,~,INSWaveResultOut ]  = INS_VNS_Match( INSA_All_k,inertialFre,otherMakersPosition_k,otherMakersN_k,visualFre,MinMatchDegree );
    
    vk = fix(j/inertialFre*visualFre)  ;
    vk = min( vk,size(otherMakers,2) );
    for i = vk_last+1:vk
        INSA_All_k = [  ];
    
        otherMakersPosition_k = otherMakers(i).Position;
        otherMakersN_k = otherMakers(i).otherMakersN ;
        visualFre = otherMakers(i).frequency;

        [ matchedDegree,~,otherMakersContinues,vns_k,VNSWaveResultOut,~,otherMakers_kNew ]  = INS_VNS_Match( INSA_All_k,inertialFre,otherMakersPosition_k,otherMakersN_k,visualFre,MinMatchDegree );
        
        if i==1290-VNSstartK  % 637    % 
            disp('');
%             for i_marker=1:otherMakers(i).otherMakersN                
 %              Draw_otherMakersContinues( otherMakersContinues,1,visualFre,vns_k );               
%             end
      %     DrawINSA_TestOut( INSA_TestOut,inertialFre,5 );
    %  DrawAllINSPosition( dataFolder,dataName1,dataName2,0,6,inertialFre,BothCutT+4.01 );
  %  DrawAllVNSPosition( otherMakersContinues,visualFre,6,i,dataFolder );
        end
      %   CheckOtherMakers( otherMakersContinues,visualFre,vns_k );
    end   
    vk_last = vk ;
    if mod(j,fix(inertialN/100))==0
        waitbar( j/inertialN,wh,sprintf('INT k = %0.0f',j+INSCutT*inertialFre ) );
    end
    if mod(j,500)==0
  %     DrawINSA_TestOut( INSA_TestOut,inertialFre,j );
       disp('');
       
    end
    if sum(~isnan(matchedDegree))~=0
       disp(''); 
    end
end

close(wh)


function CheckOtherMakers( otherMakersContinues,visualFre,vns_k )
if isempty(otherMakersContinues)
   return; 
end
otherMakersN = otherMakersContinues.otherMakersN;
dataN = otherMakersContinues.dataN;
InitialJointK  = otherMakersContinues.InitialJointK;
for i=1:otherMakersN
    
    if isnan( InitialJointK(i) ) 
       ContinuesT = dataN(i)/visualFre ;
       if  ContinuesT > 2  % 长时间连续，但未被识别的点
           fprintf( '未识别vns_k=%d, mark(%d),ContinuesT=%0.2f \n',vns_k,i,ContinuesT );
           Draw_otherMakersContinues( otherMakersContinues,i,visualFre,vns_k );
       end
    end
end

function [ HeadA,LeftHandA,RightHandA ] = ReadINSAccData( dataFolder,dataName,startK,zgStatic ) 

CalStruct = ReadCalData ( dataFolder,dataName ) ;

HeadA = CalStruct.Head.A ;
LeftHandA = CalStruct.LeftHand.A ;
RightHandA = CalStruct.RightHand.A;

HeadA(3,:) = HeadA(3,:)-zgStatic ;
LeftHandA(3,:) = LeftHandA(3,:)-zgStatic ;
RightHandA(3,:) = RightHandA(3,:)-zgStatic ;

HeadA = CutData( HeadA,startK );
LeftHandA = CutData( LeftHandA,startK );
RightHandA = CutData( RightHandA,startK );

function [ HeadP,LeftHandP,RightHandP ] = ReadINSPData( dataFolder,dataName,startK ) 

CalStruct = ReadCalData ( dataFolder,dataName ) ;

HeadP = CalStruct.Head.X ;
LeftHandP = CalStruct.LeftHand.X ;
RightHandP = CalStruct.RightHand.X;

HeadP = CutData( HeadP,startK );
LeftHandP = CutData( LeftHandP,startK );
RightHandP = CutData( RightHandP,startK );

function data = CutData( data,startK )
[M,N] = size(data);
if startK<1
   startK=1; 
end
data = data(:,startK:N);

function DrawINSA_TestOut( INSA_TestOut,inertialFre,nJoint )

INSA = INSA_TestOut.INSA;
INSA_WaveFlag_All =    INSA_TestOut.INSA_WaveFlag_All ;
INSA_V_All =    INSA_TestOut.INSA_V_All ;
INSA_waveFront_All =    INSA_TestOut.INSA_waveFront_All  ;
INSA_waveBack_All =    INSA_TestOut.INSA_waveBack_All  ;

validN = INSA_TestOut.validN ;
k_End = INSA_TestOut.timeN ;

n=nJoint;
HeadA = INSA( n*3-2:n*3,1:validN );
HeadA_WaveFlag = INSA_WaveFlag_All( n*3-2:n*3,1:validN );
HeadA_V  = INSA_V_All( n*5-4:n*5,1:validN );
HeadA_Acc_waveFront = INSA_waveFront_All( n*3-2:n*3,1:validN );
HeadA_Acc_waveBack = INSA_waveBack_All( n*3-2:n*3,1:validN );

JointName = { 'Head1','LeftHand1','RightHand1','Head2','LeftHand2','RightHand2' };

DrawWaveSearchResult( HeadA,inertialFre,HeadA_V,HeadA_WaveFlag,HeadA_Acc_waveFront,HeadA_Acc_waveBack,JointName{n},k_End );

