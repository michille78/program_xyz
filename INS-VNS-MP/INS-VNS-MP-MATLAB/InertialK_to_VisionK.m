%% xyz 2015.5.26

function [ vision_k,TimeError ] = InertialK_to_VisionK( inertial_k,otherMakersTime,inertialTime  )


global visionFre  inertialFre

MaxTimeErr = 2.5e-2 ;

inertial_T = inertialTime(inertial_k);
vision_k_Ser = fix( (inertial_k-1)*visionFre/inertialFre )+1; % 搜索起点    
vision_k_Ser = min( vision_k_Ser,length(otherMakersTime) );    

visionTime_k = otherMakersTime( vision_k_Ser );
TimeError = inertial_T-visionTime_k ;

if abs(TimeError)<1e-2
    vision_k = vision_k_Ser ;
    return;
end
if  TimeError>0
   % 惯性时间大于视觉，需要加
   vision_k_Flag = 1;
else
    % 惯性时间小于视觉，需要减
   vision_k_Flag = -1;
end


search_n = 0;
while search_n<100
    vision_k_Ser_Next = max(vision_k_Ser+1*vision_k_Flag,1) ;
    
    visionTime_k_Next = otherMakersTime( vision_k_Ser_Next );
    TimeError_Next = inertial_T-visionTime_k_Next ;
    
    if abs(TimeError_Next)<=abs(TimeError)
        TimeError = TimeError_Next ;
        vision_k_Ser = vision_k_Ser_Next ;
    else
        % OK
        
        break;
    end
    search_n = search_n+1 ;
end
vision_k = vision_k_Ser ;

if abs(TimeError)>MaxTimeErr
    fprintf( 'VisionK_to_InertialK 误差偏大 TimeError=%0.3f s， search_n= \n',TimeError );
end


