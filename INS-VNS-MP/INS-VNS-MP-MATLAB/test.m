i=0
for k=1:10
   i=i+1
   if i==2
      break; 
   end
end

return;
for i=1:INS_WaveV.N
   for j=1:VNS_WaveV.N
       if INS_WaveV.Sign(i) == VNS_WaveV.Sign(j)  % 1：都为V  -1：都为倒V
           T1Err_i_j = INS_WaveV.T1(i) - VNS_WaveV.T1(j) ;
           T2Err_i_j = INS_WaveV.T2(i) - VNS_WaveV.T2(j) ;
           EndTErr_i_j = INS_WaveV.PointT(3,i) - VNS_WaveV.PointT(3,j) ;
           
           if ~isnan(VNS_WaveV.T1Err(j)) && abs(T1Err_i_j) < reTimeErr_Big && abs(T2Err_i_j) < reTimeErr_Big              
             disp('error 2 WaveMatch_OneDim 同一个视觉点匹配成功了2个惯性点，reTimeErr_Big 设置太大！')
             VNS_WaveV.T1Err(j) = T1Err_i_j;
             VNS_WaveV.T2Err(j) = T2Err_i_j; 
             continue;
           end
           
           if abs(T1Err_i_j) < reTimeErr_Small && abs(T2Err_i_j) < reTimeErr_Small 
                % 得到一个很相近的 V 波形
                matchedDegree = matchedDegree + 0.6002;
                VNS_WaveV.T1Err(j) = T1Err_i_j;
                VNS_WaveV.T2Err(j) = T2Err_i_j; 
           elseif abs(T1Err_i_j) < reTimeErr_Big && abs(T2Err_i_j) < reTimeErr_Big 
               % 得到一个比较相近的 V 波形
               matchedDegree = matchedDegree + 0.30002;
               VNS_WaveV.T1Err(j) = T1Err_i_j;
               VNS_WaveV.T2Err(j) = T2Err_i_j;  
           end
                               
       end
   end
end