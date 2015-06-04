%% xyz 2015.6.2
%% 直接得到关节的位置
% 坐标系： 

function JointDisplacement = GetJointDisplacement( BVHStruct,JointName_k )


JointData = BVHStruct.JointData ;
if ~isfield( JointData,JointName_k )
    errordlg(sprintf('BVHStruct无成员%s',JointName_k));
    JointDisplacement = NaN;
    return;
end
if BVHStruct.isContainDisp ==0 && ~strcmp( JointName_k,'ROOT_Hips' )
    errordlg('未存储位置');
end
JointData_k = eval( sprintf('JointData.%s ;',JointName_k) );
JointDisplacement = JointData_k( :,1:3 );   % 如果有位置则肯定在前三列

