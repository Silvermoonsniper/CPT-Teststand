rem function to calculate predicted distance based on local linear regression parameters
'predicted distance at current time step is done as follows, when we get one new distance sensor measurement, we trace back
'#to past sensor measurement by L steps, L is number of datapoints which is equal to maximum correlation, then we calculate 
'# predict distance at current time step as distance data before L points plus the summation of estimated distance difference
'# for past L points using linear regression model
' the calculation is done with incremental approach to running with high effieciency.
rem input args:
rem          estimated_slope_array[]: array to store estimated slope 
rem          estimated_intercept_array[]: array to store estimated intercept
rem          estimated_parameter_pointer: pointer to update
rem         distance_data[]: distance data array
rem         controlleroutput[]: controller output array
rem         linear_coff: linea regression parameters
rem         datasize_train: number of datapoints in the buffer
rem         MAX_correlationstep: time step for maximum correlation
rem         predict_distance_pointer:pointer for incremental calculation
rem         predict_full: flag to indicate array is full
rem         estima_vel_sub[]: array to store local velocity calculated from model
rem         windowsize. size of distance data array
rem         pointer_average: pointer for distance data array
rem         local_c: pointer for controller output array 
function predict_distance(linear_coff[],estimated_slope_array[],estimated_intercept_array[],estimated_parameter_pointer,predict_distance_pointer,predict_full,estima_vel_sub[], WegInd_Digits,windowsize,pointer_average,local_c,distance_data[], controlleroutput[], datasize_train ,MAX_correlationstep)  as  float
  rem define summation of estimated distance difference for past 'max_correlationstep' datapoints
  DIM delta_s_local as float
  

  
  ' 
 
  if (pointer_average > MAX_correlationstep) then
    delta_s_local = distance_data[pointer_average-MAX_correlationstep]
         
  else
    delta_s_local = distance_data[windowsize+(pointer_average-MAX_correlationstep)]
          
  endif
      
  rem store estimated velocity into array
  if (predict_full = 0) then
    if (predict_distance_pointer <= MAX_correlationstep-1) then
      if (local_c > predict_distance_pointer) then  
        estima_vel_sub[predict_distance_pointer] = linear_coff[1] * controlleroutput[local_c-predict_distance_pointer]  + linear_coff[2]
      else
        estima_vel_sub[predict_distance_pointer] = linear_coff[1] * controlleroutput[data_sizetrain + local_c-predict_distance_pointer]  + linear_coff[2]
  
      endif
            
      inc(predict_distance_pointer)
    else
      predict_distance_pointer = 1
      rem array is full
      predict_full = 1
    endif
  endif
  if (predict_full = 0) then
   
    delta_s_local = delta_s_local + estima_vel_sub[predict_distance_pointer]
              
   
  endif    
              
        
          
  if (predict_full = 1) then
    if (predict_distance_pointer <  MAX_correlationstep-1) then 
      if (local_c > predict_distance_pointer) then  
        delta_s_local = delta_s_local - estima_vel_sub[predict_distance_pointer+ 1] + linear_coff[1] * controlleroutput[local_c-predict_distance_pointer]  + linear_coff[2]
      else
        delta_s_local = delta_s_local - estima_vel_sub[predict_distance_pointer+ 1] + linear_coff[1] * controlleroutput[data_sizetrain +local_c-predict_distance_pointer]  + linear_coff[2]
   
      endif    
           
    else
      if (local_c > predict_distance_pointer) then      
        delta_s_local = delta_s_local - estima_vel_sub[1] +linear_coff[1] * controlleroutput[data_sizetrain +local_c-predict_distance_pointer]  + linear_coff[2]
      else
        
      endif
            
          
    endif 
  endif
  rem obtain predict distance
  predict_distance = delta_s_local
endfunction
