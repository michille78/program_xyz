%% xyz 2015.4.30


%%% make:  
% 1)   norm(qv(:,i))=1
% 2) if abs(qv(1))>1e-3  qv(1)>0
% elseif abs(qv(3))>1e-3   qv(3)>0
% elseif abs(qv(2))>1e-3   qv(2)>0



function [ qv,sinHalfAfaSign ] = MakeVectorDirectionSame( qv )


%% make direction of qv  the same
sinHalfAfaSign = 1 ;
if abs( qv(1) )>1e-3 
    % qv(1)>0
   if  qv(1)<0
       sinHalfAfaSign = -1 ;
   end
else
    if abs( qv(3) )>1e-3 
       % qv(3)>0
       if  qv(3)<0
           sinHalfAfaSign = -1 ;
       end
    else
        % qv(2)>0
        if  qv(2)<0
           sinHalfAfaSign = -1 ;
       end
    end
end

qv = qv*sinHalfAfaSign ;






