
function Draw_otherMakersContinues( otherMakersContinues,i_marker,visionFre,k_vision_End )
if isempty(otherMakersContinues)
   return; 
end
[~,ConPosition_i,ConVelocity_i,ConAcc_i,AWave] = Read_otherMakersContinues_i( otherMakersContinues,i_marker );
dataN_i_P = otherMakersContinues.dataN( 1,i_marker );
DrawPVA( ConPosition_i(:,1:dataN_i_P),ConVelocity_i(:,1:dataN_i_P),ConAcc_i(:,1:dataN_i_P),visionFre,k_vision_End );

A_WaveFlag  = AWave( (14:16)-13,1:dataN_i_P);  
A_V = AWave((17:21)-13,1:dataN_i_P); 
A_Acc_waveFront = AWave((22:24)-13,1:dataN_i_P); 
A_Acc_waveBack = AWave((25:27)-13,1:dataN_i_P);   
DrawWaveSearchResult( ConAcc_i(:,1:dataN_i_P),visionFre,A_V,A_WaveFlag,A_Acc_waveFront,A_Acc_waveBack,'VNSAcc',k_vision_End );