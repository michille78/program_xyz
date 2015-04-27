%% remove some point whose plane fitting error is big
% remove the points whose error bigger than std* stdPercent

function   [ H_b_Perfected,remainNum,H_b_PerfectedStr ] = PerfectPointsOnPlane( planeFittingError,H_b,stdPercent )
if stdPercent>1.2
    stdPercent = 1.2 ;
end
if stdPercent<0.01
    stdPercent = 0.01 ;
end
   
H_b_Perfected = H_b ;
errStd = std(planeFittingError) ;
maxErr = errStd*stdPercent ;
k = 1 ;
while k<=length(planeFittingError)
   if planeFittingError(k) > maxErr 
       planeFittingError(k) = [];
       H_b_Perfected(k,:) = [];
   else
       k = k+1 ;
   end
end

remainNum = length(H_b_Perfected)/length(H_b) ;

H_b_PerfectedStr = sprintf( 'Hb perfect method: remove plane fitting error bigger than std*%0.2f \n\t\t   %0.0f(%0.2f ) data remained\n',stdPercent,length(H_b_Perfected),remainNum );
