%% xyz   2015.4.1


function BVHStruct = UpdateBVHStruct( BVHStruct )

%% BVHStruct.BVHData
data = BVHStruct.data ;
BVHFormat = BVHStruct.BVHFormat ;
BVHDataNew  = [];
BVHDataRotationNew = [];
N = length(BVHFormat);
isContainPosition = 0 ;
for k=1:N
    BVHDataNew_k = eval( sprintf('data.%s',BVHFormat{k}) ) ;
    BVHDataNew = [ BVHDataNew,BVHDataNew_k  ];
    
    BVHDataNew_k_M = size(BVHDataNew_k,2);
    if k==1
        BVHDataRotation_k = BVHDataNew_k( :,BVHDataNew_k_M-5:BVHDataNew_k_M );
    else
        BVHDataRotation_k = BVHDataNew_k( :,BVHDataNew_k_M-2:BVHDataNew_k_M );
    end
    BVHDataRotationNew = [ BVHDataRotationNew,BVHDataRotation_k ];
    if k>1 && BVHDataNew_k_M>3
       isContainPosition = 1 ; 
    end
end
BVHStruct.BVHData = BVHDataNew ;
BVHStruct.BVHDataRotation = BVHDataRotationNew ;
BVHStruct.isContainPosition = isContainPosition ;

