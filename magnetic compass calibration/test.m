

N = size(H_n_xy,1);
for k=1:N
   if H_n_xy(k,1) ==0 && H_n_xy(k,2)==0
      disp('error') 
   end
end

disp('ok')