
%% 绘制所有的INS位置

function DrawAllVNSPosition( otherMakersContinues,visionFre,otherMarkersN,k_vision_End,dataFolder )

 figFolder = sprintf( '%s\\%0.0f sec VNS',dataFolder,k_vision_End/visionFre );
 if isdir(figFolder)
     delete([figFolder,'\*'])
 else
    mkdir(figFolder); 
 end
 
ConPosition_All = cell(1,otherMarkersN);
for i_marker=1:otherMarkersN
    [~,ConPosition_i,ConVelocity_i,ConAcc_i,AWave] = Read_otherMakersContinues_i( otherMakersContinues,i_marker );
    dataN_i_P = otherMakersContinues.dataN( 1,i_marker );
    ConPosition_All{i_marker} =  ConPosition_i(:,1:dataN_i_P) ;
end
 
 figure('name','all VNS xyP')
 for i_marker=1:min(otherMarkersN,8)
    subplot(4,2,i_marker)
    plot( ConPosition_All{i_marker}(1,:),ConPosition_All{i_marker}(2,:) )
 end  
 saveas(gcf,[figFolder,'\',get(gcf,'name')])
 
   figure('name','all VNS xP')
 for i_marker=1:min(otherMarkersN,8)
    subplot(4,2,i_marker)
    visionN = size(ConPosition_All{i_marker}(3,:),2);
    time = ((1:visionN)+k_vision_End-visionN) /visionFre;
    plot( time,ConPosition_All{i_marker}(1,:) )
 end
  saveas(gcf,[figFolder,'\',get(gcf,'name')])
 
   figure('name','all VNS yP')
 for i_marker=1:min(otherMarkersN,8)
    subplot(4,2,i_marker)
    visionN = size(ConPosition_All{i_marker}(3,:),2);
    time = ((1:visionN)+k_vision_End-visionN) /visionFre;
    plot( time,ConPosition_All{i_marker}(2,:) )
 end
  saveas(gcf,[figFolder,'\',get(gcf,'name')])
 
   figure('name','all VNS zP')
 for i_marker=1:min(otherMarkersN,8)
    subplot(4,2,i_marker)
    visionN = size(ConPosition_All{i_marker}(3,:),2);
    time = ((1:visionN)+k_vision_End-visionN) /visionFre;
    plot( time,ConPosition_All{i_marker}(3,:) )
 end
  saveas(gcf,[figFolder,'\',get(gcf,'name')])