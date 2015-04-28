%% xyz 2015.4.28

%% vector_new = vector * K   [3*N]
%%% make:  
% 1)   norm(vector(:,i))=1
% 2) if abs(vector(1))>1e-3  vector(1)>0
% elseif abs(vector(3))>1e-3   vector(3)>0
% elseif abs(vector(2))>1e-3   vector(2)>0

function vector = MakeVectorDirectionSame( vector )

format long
coder.inline('never');

vector = Make_Const_N( vector,3 ) ; 
Nframes = size( vector,2 );
for k=1:Nframes
   vector(:,k)  = MakeVectorDirectionSame_One( vector(:,k) );
end

function vector = MakeVectorDirectionSame_One( vector )

vector = vector/normest(vector);

if abs( vector(1) )>1e-3 
    % vector(1)>0
   if  vector(1)<0
       vector = -vector ;
   end
else
    if abs( vector(3) )>1e-3 
       % vector(3)>0
       if  vector(3)<0
           vector = -vector ;
       end
    else
        % vector(2)>0
        if  vector(2)<0
           vector = -vector ;
       end
    end
end


