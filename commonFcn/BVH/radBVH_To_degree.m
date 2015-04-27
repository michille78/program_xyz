%% chang BVH unit from rad to degree

function BVHStruct = radBVH_To_degree( BVHStruct )
data = BVHStruct.data ;
BVHFormat = BVHStruct.BVHFormat ;
N = length(BVHFormat);
for k=1:N
    data_k = eval( sprintf('data.%s',BVHFormat{k}) ) ;
    data_k_m = size(data_k,2);
    if data_k_m==3
        data_k = data_k*180/pi ;
    else  % data_k_m==6
        data_k(:,4:6) = data_k(:,4:6)*180/pi ;
    end
    eval( sprintf('data.%s  = data_k ;',BVHFormat{k}) ) ;
        
end

BVHStruct.data = data ;


BVHStruct = UpdateBVHStruct( BVHStruct ) ;