%% xyz  2015.6.2

%% 由 MatrixData -> MatrixDataNoDisp


function MatrixDataNoDisp = GetNoDispMatrixData( MatrixData,isContainDisp )
MatrixDataNoDisp = MatrixData ;
if isContainDisp==0
    return;
end

k=12 ;  % 第一个需要删除的位置是 [10 11 12]
while k<=size(MatrixDataNoDisp,2)
    MatrixDataNoDisp( :,k-2:k ) =[] ;   % 每次删三列
    
    k = k+3;  % 注意：本来每次进6个，由于上次删了3，只进3
end