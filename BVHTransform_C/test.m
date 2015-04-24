
BVH_Nao2 = importdata('BVH_Nao2.mat');
BVHData2 = BVH_Nao2.BVHData ;
BVH_Nao1 = importdata('BVH_Nao2.mat');
BVHData1 = BVH_Nao1.BVHData ;
BVH_Nao = importdata('BVH_Nao.mat');
BVHData = BVH_Nao.BVHData ;

BVHDataErr2 = BVHData - BVHData2 ;
BVHDataErr1 = BVHData - BVHData1 ;
BVHDataErrSum1 = sum(sum(abs(BVHDataErr1)));
BVHDataErrSum2 = sum(sum(abs(BVHDataErr2)));

NaoData_degree = importdata('NaoData_degree.mat');
NaoData = importdata('NaoData.mat');

NaoData_err = NaoData - NaoData_degree ;


disp('ok')
