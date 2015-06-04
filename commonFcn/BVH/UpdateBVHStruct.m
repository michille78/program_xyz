%% xyz   2015.4.1


function BVHStruct = UpdateBVHStruct( BVHStruct )

%% BVHStruct.BVHData
JointData = BVHStruct.JointData ;
JointName = BVHStruct.JointName ;
MatrixData  = [];
N = length(JointName);
for k=1:N
    BVHDataNew_k = eval( sprintf('JointData.%s',JointName{k}) ) ;
    MatrixData = [ MatrixData,BVHDataNew_k  ];
    
end
BVHStruct.MatrixData = MatrixData ;
BVHStruct.MatrixDataNoDisp = GetNoDispMatrixData( MatrixData,BVHStruct.isContainDisp );
BVHStruct.Frames = size(MatrixData,1);
