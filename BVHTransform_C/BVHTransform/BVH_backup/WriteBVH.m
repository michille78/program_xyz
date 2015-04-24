%% xyz 2015.4.1



function WriteBVH( BVHStruct,dataFolder,dataName )

BVHHeadStr = BVHStruct.BVHHeadStr ;
BVHDataRotation = BVHStruct.BVHDataRotation ;

% BVHFormat = BVHStruct.BVHFormat ;
% for i=1:length(BVHFormat)
%     if strcmp(BVHFormat{i},'LeftLeg')
%        disp('LeftLeg'); 
%     end
% end


fID = fopen( [dataFolder,'\',dataName,'.bvh'],'w' );
fprintf( fID,'%s',BVHHeadStr );

N = size( BVHDataRotation,1 );
for k=1:N    
    if k<N
        fprintf( fID,'%7.2f ',BVHDataRotation(k,:) );
    else
        fprintf( fID,'%7.2f',BVHDataRotation(k,:) );
    end
    if k<N
        fprintf( fID,'\n' );
    end
end

fclose(fID);