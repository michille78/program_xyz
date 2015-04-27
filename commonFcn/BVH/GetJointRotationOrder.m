%% xyz 2015.4.9
% RotationOrder = 'XYZ' / 'YXZ'...

function RotationOrder = GetJointRotationOrder( BVHStruct,JointName )
RotationOrder = '000';

BVHHeadStr = BVHStruct.BVHHeadStr ;
JointName  = sprintf('%s',JointName); % 得加一个空格
JointStasrtN = strfind( BVHHeadStr,JointName );
if BVHHeadStr(JointStasrtN+length(JointName)) ~= 13
    JointStasrtN = [];      % 不会从 'RightHandMiddle1' 中读出 'RightHandMiddle'
end
if isempty(JointStasrtN)
    RotationOrder = [];
    return;
end
BVHHeadStr_new = BVHHeadStr( JointStasrtN:length(BVHHeadStr) );
Joint_CHANNELS_StartN = strfind( BVHHeadStr_new,'rotation' );
RotationOrder(1) = BVHHeadStr_new( Joint_CHANNELS_StartN(1)-1 );
RotationOrder(2) = BVHHeadStr_new( Joint_CHANNELS_StartN(2)-1 );
RotationOrder(3) = BVHHeadStr_new( Joint_CHANNELS_StartN(3)-1 );