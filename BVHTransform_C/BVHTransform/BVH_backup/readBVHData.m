%% 
% 输出时保留 度 为单位
function  BVHStruct = readBVHData ( dataFolder,dataName )

% dataName = 'BVHData';
% dataFolder = 'E:\data_xyz_noitom\BVHTransformTestData\test1';
BVHFilePath = [dataFolder,'\',dataName,'.bvh'] ;
numberStartLine = 353 ;
if ~exist([dataFolder,'\',dataName,'.mat'],'file')
    disp('read BVH')
%  dbstop in readBVH_Format
    [ BVHFormat,Frames,Frame_Time ]  = readBVH_Format( BVHFilePath,numberStartLine ) ; 
    [ BVHHeadStr,isContainPosition ]  = readBVH_HeadStr( BVHFilePath,numberStartLine ) ;
    if strcmp(BVHFormat{1,1},'ROOT Hips')
        BVHFormat{1,1} = 'ROOT_Hips';
    else
        errordlg('BVHFormat{1,1} not ROOT Hips');
    end
    disp('In reading BVH data...');
    BVHData = readBVH_Data( BVHFilePath,numberStartLine ) ;  
    if size(BVHData,1)~=Frames
       errordlg( sprintf(' Frames = %d, but read %d ',Frames,size(BVHData,1)) ); 
    else
        display( sprintf( 'Frames = %d',Frames ));
    end
    if isContainPosition
%         dbstop in GetRotationData
        BVHDataRotation = GetRotationData(BVHData);
    else
        BVHDataRotation = BVHData ;
    end
    
    BVHStruct.BVHData = BVHData ;
    BVHStruct.BVHDataRotation = BVHDataRotation ;
    BVHStruct.BVHFormat = BVHFormat ;
    BVHStruct.BVHHeadStr = BVHHeadStr ;
    BVHStruct.Frame_Time = Frame_Time ;
    
    BVHFormat_N = size(BVHFormat,1);
    skeleten_n_sart = 1 ;
    isContainPosition = 0;
    for k = 1:BVHFormat_N
        JointName = BVHFormat{k,1};
        % 只有Hip保留位置
        if k==1            
            skeleten_n_end = skeleten_n_sart+5 ;            
        else            
            skeleten_n_end = skeleten_n_sart+2 ;           
        end
        eval( sprintf('BVHStruct.data.%s = BVHDataRotation( :,%d:%d ) ;',JointName,skeleten_n_sart,skeleten_n_end) );
        if k==1
            skeleten_n_sart = skeleten_n_sart+6 ;
        else
            skeleten_n_sart = skeleten_n_sart+3 ;
        end
        if k>1 && eval( sprintf( 'length( BVHStruct.data.%s )',JointName ) )>3
            isContainPosition = 1 ;
        end
    end
     BVHStruct.isContainPosition = isContainPosition ;
    save( [dataFolder,'\',dataName,'.mat'],'BVHStruct'  )    
else
    BVHStruct = importdata( [dataFolder,'\',dataName,'.mat'] );
    disp('import BVHStruct')
end

%% 只保留根节点的位置

function BVHDataRotation = GetRotationData(BVHData)
N = size(BVHData,1);
M = size(BVHData,2)/6 ;
BVHDataRotation = zeros(N,M*3+3);
BVHDataRotation(:,1:6) = BVHData(:,1:6) ;  % 只根节点保留位置
for i=2:M
    BVHDataRotation(:,i*3+1:i*3+3) = BVHData( :,(i-1)*6+4:i*6 );
end
% for k=2:N
%     BVHData_k = BVHData(k,:);
%     for i=1:M
%         BVHData_k_i = BVHData_k( 1+(i-1)*6:i*6 );
%         BVHDataRotation(k,1+i*3:(i+1)*3) = BVHData_k_i(4:6) ; % 注意Hip是6个数
%     end
% end

function [ SkeletenBVH,SkeletenBVH_Order ] = SearchSkeletenBVH( skeleten,BVHFormat,BVH )
N = size(BVHFormat,1);
for k=1:N
    if strcmp( skeleten,BVHFormat{k,1} )
        SkeletenBVH_Order = [ BVHFormat{k,2},BVHFormat{k,3} ];
        SkeletenBVH = BVH( :,SkeletenBVH_Order(1):SkeletenBVH_Order(2) );
    end
end

function BVH = readBVH_Data( filePath,numberStartLine )


fid = fopen(filePath,'r' ) ;
BVH = [] ;  % N*3
n = 1 ;
k = 0 ;
while ~feof(fid)
   tline = fgetl(fid) ; 
   if n>=numberStartLine
       lineData = textscan( tline,'%f' ) ;
       k = k+1 ;
       BVH = [ BVH ; lineData{1}' ];
   end
   n = n+1 ;
   
end
fclose(fid);

function [BVHHeadStr,isContainPosition] = readBVH_HeadStr( filePath,numberStartLine )
isContainPosition = 0;
fid = fopen(filePath,'r' ) ;
BVHHeadStr = '';
line_n = 1 ;
CHANNELS_n=0;
isReplaceFlag = 0 ;
while ~feof(fid) && line_n < numberStartLine
   lineStr = fgets(fid) ; 
   
   if ~isempty(lineStr)
       [ lineStrNew,spaceStr ] = FrontSpace( lineStr ) ;
       linet = textscan(lineStr,'%s');
       if length(linet{1})>1 && strcmp( linet{1}{1},'CHANNELS' )
           CHANNELS_n = CHANNELS_n+1 ;
           if CHANNELS_n>1
                lineStrNew = 'CHANNELS 3 Yrotation Xrotation Zrotation ';
                lineStr = sprintf( '%s%s',spaceStr,lineStrNew );  % 在最后多放了一个空格
                isReplaceFlag = 1 ;
                isContainPosition=1;
           end
       end
        if isReplaceFlag==1 && ~strcmp( linet{1}{1},'CHANNELS' )
            BVHHeadStr = sprintf('%s\n%s',BVHHeadStr,lineStr);
            isReplaceFlag = 0; 
        else
            BVHHeadStr = sprintf('%s%s',BVHHeadStr,lineStr);
            
        end       
        BVHHeadStr(length(BVHHeadStr)) = '';
   end
   line_n = line_n+1 ;   
   
end
fclose(fid);

function [ lineStrNew,spaceStr ] = FrontSpace( lineStr )
N = length(lineStr);
spaceN = 0;
for k=1:N
   if strcmp(lineStr(k),' ') 
       spaceN = spaceN+1 ;
   else
       break;
   end
end
spaceStr = lineStr(1:spaceN);
lineStrNew = lineStr( spaceN+1:N );

function [ BVHFormat,Frames,Frame_Time ] = readBVH_Format( filePath,numberStartLine )
fid = fopen(filePath,'r' ) ;

BVHFormat = cell(10,1);
k = 1 ;
BVHFormat{k,1} = 'ROOT Hips';
line_n = 1 ;
while ~feof(fid) && line_n < numberStartLine
   tline = fgetl(fid) ; 
   if ~isempty(tline)
       lineData = textscan( tline,'%s' ) ;
       if strcmp( lineData{1}{1},'JOINT' )
           k = k+1 ;
           BVHFormat{k,1} =  lineData{1}{2} ;
       end
       if strcmp( lineData{1}{1},'Frames:' )
           Frames = str2double( lineData{1}{2} );
       end
       if strcmp( lineData{1}{1},'Frame' ) && strcmp( lineData{1}{2},'Time:' )
           Frame_Time = str2double( lineData{1}{3} );
       end
   end
   line_n = line_n+1 ;   
end
fclose(fid);
N_BVH = k ;
BVHFormat = BVHFormat( 1:N_BVH,: ) ;
