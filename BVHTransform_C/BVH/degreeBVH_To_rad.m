%% chang BVH unit from degree to rad

function BVHStruct = degreeBVH_To_rad( BVHStruct )

data = BVHStruct.data ;
BVHFormat = BVHStruct.BVHFormat ;
N = length(BVHFormat);
for k=1:N
    data_k = eval( sprintf('data.%s',BVHFormat{k}) ) ;
    data_k_m = size(data_k,2);
    if data_k_m==3
        data_k = data_k*pi/180 ;
    else  % data_k_m==6
        data_k(:,4:6) = data_k(:,4:6)*pi/180 ;
    end
    eval( sprintf('data.%s  = data_k ;',BVHFormat{k}) ) ;
        
end

BVHStruct.data = data ;


BVHStruct = UpdateBVHStruct( BVHStruct ) ;