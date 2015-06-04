%% xyz   2015.6.2

function DrawAllBVHJoint( BVHStruct )


JointName = BVHStruct.JointName ;
JointN = length(JointName);

for k=1:JointN
    fh = DrawBVHJoint( BVHStruct,JointName{k} ) ;
end

