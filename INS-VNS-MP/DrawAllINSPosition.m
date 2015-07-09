%% 绘制所有的INS位置

function DrawAllINSPosition( dataFolder,dataName1,dataName2,StartT,StopT,fre,CutT )

 figFolder = sprintf( '%s\\%0.0f-%0.0f sec INS',dataFolder,StartT,StopT );
 if isdir(figFolder)
     delete([figFolder,'\*'])
 else
    mkdir(figFolder); 
 end
     
CutK = CutT*fre+1;
[ HeadP1,LeftHandP1,RightHandP1 ] = ReadINSPData( dataFolder,dataName1,CutK ) ;
[M,N] = size(HeadP1);
if isempty(dataName2)
    HeadP2 = NaN(M,N);
    LeftHandP2 = NaN(M,N);
    RightHandP2 = NaN(M,N);
else
    [ HeadP2,LeftHandP2,RightHandP2 ] = ReadINSPData( dataFolder,dataName2,CutK ) ;    
end

INSPosition = [ HeadP1;LeftHandP1;RightHandP1;HeadP2;LeftHandP2;RightHandP2 ];



 StartK = StartT*fre ;
 if StartK<1     
     StartK=1;
 end
 StopK = StopT*fre ;
 time = (StartK:StopK)/fre;
 INSPosition = INSPosition( :,StartK:StopK );
 
 figure('name','all INS xyP')
 subplot(3,2,1)
 plot( INSPosition(1,:),INSPosition(2,:) )
  subplot(3,2,2)
 plot( INSPosition(4,:),INSPosition(5,:) )
  subplot(3,2,3)
 plot( INSPosition(7,:),INSPosition(8,:) )
  subplot(3,2,4)
 plot( INSPosition(10,:),INSPosition(11,:) )
  subplot(3,2,5)
 plot( INSPosition(13,:),INSPosition(14,:) )
  subplot(3,2,6)
 plot( INSPosition(16,:),INSPosition(17,:) )
 
 saveas(gcf,[figFolder,'\',get(gcf,'name')])
 
   figure('name','all INS xP')
 subplot(3,2,1)
 plot( time,INSPosition(1,:) )
  subplot(3,2,2)
 plot( time,INSPosition(4,:) )
  subplot(3,2,3)
 plot( time,INSPosition(7,:) )
  subplot(3,2,4)
 plot( time,INSPosition(10,:) )
  subplot(3,2,5)
 plot( time,INSPosition(13,:) )
  subplot(3,2,6)
 plot( time,INSPosition(15,:) )
 
  saveas(gcf,[figFolder,'\',get(gcf,'name')])
  
   figure('name','all INS yP')
 subplot(3,2,1)
 plot( time,INSPosition(2,:) )
  subplot(3,2,2)
 plot( time,INSPosition(5,:) )
  subplot(3,2,3)
 plot( time,INSPosition(8,:) )
  subplot(3,2,4)
 plot( time,INSPosition(11,:) )
  subplot(3,2,5)
 plot( time,INSPosition(14,:) )
  subplot(3,2,6)
 plot( time,INSPosition(17,:) )
 
  saveas(gcf,[figFolder,'\',get(gcf,'name')])
  
  figure('name','all INS zP')
 subplot(3,2,1)
 plot( time,INSPosition(3,:) )
  subplot(3,2,2)
 plot( time,INSPosition(6,:) )
  subplot(3,2,3)
 plot( time,INSPosition(9,:) )
  subplot(3,2,4)
 plot( time,INSPosition(12,:) )
  subplot(3,2,5)
 plot( time,INSPosition(15,:) )
  subplot(3,2,6)
 plot( time,INSPosition(18,:) )
 
  saveas(gcf,[figFolder,'\',get(gcf,'name')])
  
  
  
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
data = data(:,startK:N);