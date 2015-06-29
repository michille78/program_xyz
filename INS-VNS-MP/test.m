function otherMakersContinues = test(otherMakersN)
otherMakersN = 5;

fieldNames = [ 'ConP01';'ConP02';'ConP03';'ConP04';'ConP05';'ConP06';'ConP07';'ConP08';'ConP09';'ConP10';...
    'ConP11';'ConP12';'ConP13';'ConP14';'ConP15';'ConP16';'ConP17';'ConP18';'ConP19';'ConP20' ];

otherMakersContinues.fieldNames(1,:) = 12;

for i=1:otherMakersN
   eval(['otherMakersContinues.ConPosition_',mat2str(i),' = i;']) ;
end

names = fieldnames( otherMakersContinues );

disp('')
