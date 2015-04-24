

BVHStruct_new = importdata('BVHStruct_new.mat');
BVHData_new = BVHStruct_new.BVHData ;

BVHStruct_Original = importdata('BVHStruct_Original.mat');
BVHData_original = BVHStruct_Original.BVHData ;

BVH_Nao1 = importdata('BVH_Nao1.mat');
BVHData1 = BVH_Nao1.BVHData ;

BVHDataErr1 = BVHData_new - BVHData1 ;
BVHDataErrSum1 = sum(sum(abs(BVHDataErr1)));




LeftLeg = BVHStruct.data.LeftLeg;
figure
plot( LeftLeg(:,1),'r' );
hold on
plot( LeftLeg(:,2),'b' );
hold on
plot( LeftLeg(:,3),'k' );
legend('y','x','z')

RightLeg = BVHStruct.data.RightLeg;
figure('name','RightLeg')
plot( RightLeg(:,1),'r' );
hold on
plot( RightLeg(:,2),'b' );
hold on
plot( RightLeg(:,3),'k' );
legend('y','x','z')
