%% 读取ＢＶＨ数据到结构体　BVHStruct
% MatrixData ： [ N*354 ]  每一行为一帧
% MatrixDataNoDisp ：[N*180]  只有Hip有位置
% BVHHeadStr： [1*N] char BVH头字符串
% isContainDisp： 0/1 BVH中 Hip 之外的节点是否包含位置信息
% JointName ： cell [N*1]   JointName{i,1}存关节名  
% JointRotation： JointRotation.JointName{i} 存欧拉角旋转顺序
% BVHStruct.JointData  按关节存储数据 [N*3]  或  [N*6]
% Frames： 帧数
% Frame_Time： 周期 秒
% BVHHeadStr_NoDisp： [1*N] char 除Hip外无Displacement的 BVH 头字符串
% BVHStruct.JointData  按关节存储数据 [N*3]  或  [N*6](前三欧拉角，后三位移)

% 位移单位： cm
% 角度单位： degree

%% BVH 数据 坐标系
% reference hip 用 北天东 坐标系
% 其他节点用 左上前 坐标系

% 输出时保留 度 为单位
function  BVHStruct = readBVHData ( dataFolder,dataName )

%  dataName = 'BVHData';
%  dataFolder = 'E:\data_xyz\Hybrid Motion Capture Data\5.28\5.28-head1';
BVHFilePath = [dataFolder,'\',dataName,'.bvh'] ;

if ~exist([dataFolder,'\',dataName,'.mat'],'file')
% if 1
    if ~exist(BVHFilePath,'file')
       errordlg(sprintf('不存在BVH文件 %s',BVHFilePath));
       return;
    end
    disp('read BVH')
    % dbstop in GetNumberStartLine
    [ BVHHeadStr,numberStartLine ] = GetNumberStartLine( BVHFilePath );
%  dbstop in readBVH_Format
    [ JointName,JointRotation,Frames,Frame_Time,isContainDisp ]  = readBVH_Format( BVHFilePath,numberStartLine ) ; 
    BVHHeadStr_NoDisp = GetBVHHeadStr_NoDisp( BVHHeadStr );
    
%     [ BVHHeadStr_NoDisp,isContainDisp ]  = readBVH_HeadStr( BVHFilePath,numberStartLine ) ;
    if strcmp(JointName{1,1},'ROOT Hips')
        JointName{1,1} = 'ROOT_Hips';
    else
        errordlg('JointName{1,1} not ROOT Hips');
    end
    disp('In reading BVH data...');
    MatrixData = readBVH_Data( BVHFilePath,numberStartLine ) ;  
    if size(MatrixData,1)~=Frames
       errordlg( sprintf(' Frames = %d, but read %d ',Frames,size(MatrixData,1)) ); 
    else
        display( sprintf( 'Frames = %d',Frames ));
    end
    MatrixDataNoDisp = GetNoDispMatrixData( MatrixData,isContainDisp ) ;

    
    BVHStruct.MatrixData = MatrixData ;
    BVHStruct.MatrixDataNoDisp = MatrixDataNoDisp ;
    BVHStruct.JointName = JointName ;
    BVHStruct.JointRotation = JointRotation ;
    BVHStruct.BVHHeadStr = BVHHeadStr ;
    BVHStruct.Frame_Time = Frame_Time ;
    BVHStruct.isContainDisp = isContainDisp ;
    BVHStruct.BVHHeadStr_NoDisp = BVHHeadStr_NoDisp ;
    BVHStruct.Frames = Frames ;
    
    BVHFormat_N = size(JointName,1);
    skeleten_n_sart = 1 ;
    %% BVHStruct.JointData  按关节存储数据 [N*3]  或  [N*6]
    for k = 1:BVHFormat_N
        JointName_k = JointName{k,1};        
        
        if isContainDisp==1
            % 每个关节都有位置
            skeleten_n_end = skeleten_n_sart+5 ;
        else
            % 除 Hip外无位置信息
            if k==1            
                skeleten_n_end = skeleten_n_sart+5 ;       % 只有Hip保留位置       
            else            
                skeleten_n_end = skeleten_n_sart+2 ;           
            end
        end
        eval( sprintf('BVHStruct.JointData.%s = MatrixData( :,%d:%d ) ;',JointName_k,skeleten_n_sart,skeleten_n_end) );
        skeleten_n_sart = skeleten_n_end + 1 ;
    end
     
    save( [dataFolder,'\',dataName,'.mat'],'BVHStruct'  )    
else
    BVHStruct = importdata( [dataFolder,'\',dataName,'.mat'] );
    disp('没读BVH，直接导入mat结果。 import BVHStruct')
end



function BVHHeadStr_NoDisp = GetBVHHeadStr_NoDisp( BVHHeadStr )

BVHHeadStr_NoDisp = strrep( BVHHeadStr,'CHANNELS 6 Xposition Yposition Zposition','CHANNELS 3' );
k = strfind( BVHHeadStr_NoDisp,'CHANNELS 3');
k1 = k(1);
BVHHeadStr_NoDisp(k1:length('CHANNELS 3')+k1-1) = '';
BVHHeadStr_NoDisp = sprintf( '%s%s%s',BVHHeadStr_NoDisp(1:k1-1),...
    'CHANNELS 6 Xposition Yposition Zposition',BVHHeadStr_NoDisp(k1:length(BVHHeadStr_NoDisp)) );

function [ SkeletenBVH,SkeletenBVH_Order ] = SearchSkeletenBVH( skeleten,JointName,BVH )
N = size(JointName,1);
for k=1:N
    if strcmp( skeleten,JointName{k,1} )
        SkeletenBVH_Order = [ JointName{k,2},JointName{k,3} ];
        SkeletenBVH = BVH( :,SkeletenBVH_Order(1):SkeletenBVH_Order(2) );
    end
end

%% 直接读取的 BVH 数字
% MatrixData ： [ N*354 ]  每一行为一帧
function MatrixData = readBVH_Data( filePath,numberStartLine )


fid = fopen(filePath,'r' ) ;
MatrixData = [] ;  % N*3
n = 1 ;
k = 0 ;
while ~feof(fid)
   tline = fgetl(fid) ; 
   if n>=numberStartLine
       lineData = textscan( tline,'%f' ) ;
       k = k+1 ;
       MatrixData = [ MatrixData ; lineData{1}' ];
   end
   n = n+1 ;
   
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

%% BVH数据格式
% JointName ： cell [N*1]   JointName{i}存关节名
% Frames： 帧数
% Frame_Time： 周期 秒
function [ JointName,JointRotation,Frames,Frame_Time,isContainDisp ] = readBVH_Format( filePath,numberStartLine )
fid = fopen(filePath,'r' ) ;

JointName = cell(10,1);
JointRotation = struct;

k = 1 ;
JointName{k,1} = 'ROOT Hips';
line_n = 1 ;
isContainDisp=  0 ;
while ~feof(fid) && line_n < numberStartLine
   tline = fgetl(fid) ; 
   if ~isempty(tline)
       lineData = textscan( tline,'%s' ) ;
       if ~isempty(lineData{1}) && strcmp( lineData{1}{1},'JOINT' )
           k = k+1 ;
           JointName{k,1} =  lineData{1}{2} ;
       end
       if ~isempty(lineData{1}) && strcmp( lineData{1}{1},'CHANNELS' )
           % 读取 JointName{k,1} 的旋转顺序
           if strcmp(JointName{k,1} ,'ROOT Hips')
               JointName_k = 'ROOT_Hips';
           else
               JointName_k = JointName{k,1} ;
           end
           order = [ lineData{1}{6}(1),lineData{1}{7}(1),lineData{1}{8}(1) ] ;
           eval( sprintf('JointRotation.%s = order; ',...
               JointName_k) )  ;
           if k>1 && length(lineData{1})==8
               isContainDisp = 1 ;
           end
       end
       if ~isempty(lineData{1}) && strcmp( lineData{1}{1},'Frames:' )
           Frames = str2double( lineData{1}{2} );
       end
       if ~isempty(lineData{1}) &&  strcmp( lineData{1}{1},'Frame' ) && strcmp( lineData{1}{2},'Time:' )
           Frame_Time = str2double( lineData{1}{3} );
       end
   end
   line_n = line_n+1 ;   
end
fclose(fid);
N_BVH = k ;
JointName = JointName( 1:N_BVH,: ) ;

%% 第一行数据的行序号
% BVHHeadStr： BVH 头字符串
% numberStartLine： 第一行数字的行序号
function [ BVHHeadStr,numberStartLine ] = GetNumberStartLine( BVHFilePath )
fid = fopen(BVHFilePath,'r' ) ;
line_n = 0 ;
numberStartLine = 0;
BVHHeadStr = '';
while ~feof(fid)  &&  line_n<360
    tline = fgetl(fid) ; 
    line_n = line_n+1 ;  
    if ~isempty(tline)
        lineData = textscan( tline,'%s' ) ;     
        if ~isempty(lineData{1}) && ~isnan( str2double( lineData{1}{1} ))
            % 是数字
            numberStartLine = line_n ;
            break;
        else
            % 不是数字
            BVHHeadStr = sprintf('%s\n%s',BVHHeadStr,tline);
        end
    else
        BVHHeadStr = sprintf('%s\n ',BVHHeadStr);
    end
    
end
