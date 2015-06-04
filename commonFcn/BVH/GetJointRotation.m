%% xyz 2015.6.2
%% 直接得到关节的位置
% 坐标系： 

function JointRotation = GetJointRotation( BVHStruct,JointName_k )


JointData = BVHStruct.JointData ;
if ~isfield( JointData,JointName_k )
    errordlg(sprintf('BVHStruct无成员%s',JointName_k));
    JointRotation = NaN;
    return;
end
JointData_k = eval( sprintf('JointData.%s ;',JointName_k) );
if size(JointData_k,2)==3;
    JointRotation = JointData_k( :,1:3 );   
else
    JointRotation = JointData_k( :,4:6 );   % 如果有位置则在后三列
end




