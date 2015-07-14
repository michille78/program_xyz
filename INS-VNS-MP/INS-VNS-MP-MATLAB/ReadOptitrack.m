%% xyz  2015.6.25

function otherMakers = ReadOptitrack( dataFolder,dataName )
if exist([dataFolder,'\otherMakers.mat'],'file')
    otherMakers = importdata([dataFolder,'\otherMakers.mat']);
    return;
end

CalFilePath = [dataFolder,'\',dataName];
fid = fopen(CalFilePath,'r' ) ;
line_n = 0 ;
k = 0;
otherMakers = struct;
while ~feof(fid) 
    tline = fgetl(fid) ; 
    line_n = line_n+1 ;  
    if ~isempty(tline)
        lineData = textscan( tline,'%s' ) ;     
        if length(lineData{1}) == 11 && ~isempty(str2double(lineData{1}{1}) )  && ~isnan(str2double(lineData{1}{1}) )
            k = k+1;
            time = str2double(lineData{1}{1});
            otherMakersN = str2double(lineData{1}{2});
            
           
           if otherMakersN>0
               otherMakers_Data_k = zeros(3,otherMakersN);
               for i=1:otherMakersN
                   for j=1:3
                       otherMakers_Data_k(j,i) = str2double(lineData{1}{i*3+j-1}) ;
                   end
                end
           else
               otherMakers_Data_k = [];
           end
           otherMakers(k).otherMakersN = otherMakersN ;
           otherMakers(k).Position = otherMakers_Data_k ;
           otherMakers(k).time = time;
        end
                      
    end
end

save( [dataFolder,'\otherMakers.mat'],'otherMakers' );
    
