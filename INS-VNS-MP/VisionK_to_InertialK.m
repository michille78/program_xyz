%% xyz 2015.5.26

function  inertial_k = VisionK_to_InertialK( vision_k )

global otherMakersTime  inertialTime 
global visionFre  inertialFre

MaxTimeErr = 1.9e-2 ;

vision_T = otherMakersTime(vision_k);
inertial_k_Ser = fix( (vision_k-1)*inertialFre/visionFre )+1; % 搜索起点
    
inertial_k_Ser = min( inertial_k_Ser,length(inertialTime) );
inerTime_k = inertialTime( inertial_k_Ser );
TimeError = inerTime_k-vision_T ;

if abs(TimeError)<1e-2
    inertial_k = inertial_k_Ser ;
    return;
end
if  TimeError>0
   % 惯性时间大于视觉，需要减
   inertial_k_Flag = -1 ;
else 
    % 惯性时间小于视觉，需要加
   inertial_k_Flag = 1 ;
end

search_n = 0;
while search_n<100
    
    inertial_k_SerNext = max(inertial_k_Ser+1*inertial_k_Flag,1) ;
    
    inerTime_k_Next = inertialTime( inertial_k_SerNext );
    TimeError_Next = inerTime_k_Next-vision_T ;
    if abs(TimeError_Next) <= abs(TimeError)
       TimeError = TimeError_Next ;
       inertial_k_Ser = inertial_k_SerNext ;
    else
        % OK               
        break;
    end
    search_n = search_n+1 ;
end
inertial_k = inertial_k_Ser ;

if abs(TimeError)>MaxTimeErr 
    fprintf( 'VisionK_to_InertialK 误差偏大 TimeError=%0.3f s， search_n = %d  \n',TimeError,search_n );
end





